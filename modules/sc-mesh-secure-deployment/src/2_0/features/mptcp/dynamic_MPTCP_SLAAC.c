#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <netinet/ip6.h>
#include <linux/if_ether.h>
#include <pcap.h>

#define DEBUG 1
#define BUFFER_SIZE 65536
#define END_HOST_INTERFACE "wlan1"

void get_socket_info(unsigned char *, int, struct sockaddr_in6 *, struct sockaddr_in6 *, bool *);
char* mac_to_eui64(char * const, char *);
void derive_mesh_exit_node_MPTCP_endpoint_addresses(struct sockaddr_in6, int, char **, char [][INET6_ADDRSTRLEN]);
void ipv6_address_prefix(struct in6_addr, char *);
void ipv6_to_str_expanded(char *, const struct in6_addr *);
void mac_from_prefix(const char *, char *dst_addr_mac);
void packet_handler(u_char *user, const struct pcap_pkthdr *h, const u_char *bytes);


// Structure to pass to packet_handler
struct argc_argv
{
	int argc;
	char **argv;
};


// Global file pointer for file access
FILE *fp = NULL;


// Considerations:
// 	a. End-host is connected to comms. device over $END_HOST_INTERFACE
// 	b. End-host obtains SLAAC prefix starting with "fdff" from the comms. device
// Usage: sudo ./tcp_syn_packet_sniffer_eth_capture_MPTCP_SLAAC "WiFi-prefix" "HaLow-prefix" [...]
int main(int argc, char **argv)
{
	struct argc_argv argc_argv_data;
	argc_argv_data.argc = argc;
	argc_argv_data.argv = argv;

	//char mesh_exit_node_endpoint_addresses[argc - 1][INET6_ADDRSTRLEN];
	//struct sockaddr_in6 source, dest, dest_original;
	//bool syn_flag;


	char errbuf[PCAP_ERRBUF_SIZE];
    	pcap_t *handle;

    	// Open the network interface for packet capture
    	handle = pcap_open_live(END_HOST_INTERFACE, BUFSIZ, 1, 1000, errbuf); //$END_HOST_INTERFACE hardcoded for now
    	if (handle == NULL) {
    	    fprintf(stderr, "Could not open device %s: %s\n", END_HOST_INTERFACE, errbuf);
    	    return 2;
    	}

	
    	// Compile and apply the IPv6 filter
    	struct bpf_program fp_bpf;
    	char filter_exp[] = "ip6 and tcp";
    	if (pcap_compile(handle, &fp_bpf, filter_exp, 0, PCAP_NETMASK_UNKNOWN) == -1) {
    	    fprintf(stderr, "Couldn't parse filter %s: %s\n", filter_exp, pcap_geterr(handle));
    	    return 2;
    	}

    	if (pcap_setfilter(handle, &fp_bpf) == -1) {
    	    fprintf(stderr, "Couldn't install filter %s: %s\n", filter_exp, pcap_geterr(handle));
    	    return 2;
    	}


	// Redirect all TCP packets from connected end-host to proxy client at port "1080"
	fp = fopen("iptablerules_redir_mptcp_slaac.sh", "w");
	fprintf(fp,
	                 "ip6tables -t nat -N SSREDIR\
			 \nip6tables -t nat -A PREROUTING -p tcp -j SSREDIR\
			 \nip6tables -t nat -A SSREDIR -p tcp -s fdff::/16 -j REDIRECT --to-ports 1080"
	       );
	fflush(fp);
	fclose(fp);
	system("bash iptablerules_redir_mptcp_slaac.sh");


	//Block all incoming TCP packets from connected end-host
	fp = fopen("iptablerules_block_mptcp_slaac.sh", "w");
	fprintf(fp,
	                 "ip6tables -t filter -A INPUT -p tcp -s fdff::0/16 -j DROP"
	       );
	fflush(fp);
	fclose(fp);
	system("bash iptablerules_block_mptcp_slaac.sh");



	// Start the proxy server
	fp = fopen("ss-server_mptcp_slaac.json", "w");
	fprintf(fp,
                         "{\
                         \n\t\"server\" : [\"[::0]\", \"0.0.0.0\"],\
			 \n\t\"mode\":\"tcp\",\
			 \n\t\"server_port\" : 8388,\
			 \n\t\"local_address\":\"::0\",\
			 \n\t\"local_port\" : 1080,\
			 \n\t\"password\" : \"sai\",\
			 \n\t\"timeout\" : 300,\
			 \n\t\"method\" : \"aes-256-cfb\"\
                         \n}"
               );
	fflush(fp);
        fclose(fp);
	system("ss-server -c ss-server_mptcp_slaac.json &");


	// Capture packets and call the packet_handler function for each captured packet
	
	pcap_loop(handle, 0, packet_handler, (u_char *)&argc_argv_data);

	//Free fp
        pcap_freecode(&fp_bpf);

        // Close the handle
        pcap_close(handle);

	
	return 0;
}

void get_socket_info(unsigned char *buffer, int data_size, struct sockaddr_in6 *source, struct sockaddr_in6 *dest, bool *syn_flag)
{
	memset(source, 0, sizeof(struct sockaddr_in6));
	memset(dest, 0, sizeof(struct sockaddr_in6));

	// IPv6 header
	struct ip6_hdr *ip6h = (struct ip6_hdr *)(buffer + sizeof(struct ethhdr));
	source->sin6_addr = ip6h->ip6_src;
	dest->sin6_addr = ip6h->ip6_dst;

	// Skip extension headers (if any), until next header is TCP/ICMPv6
	int header_length = sizeof(struct ethhdr) + sizeof(struct ip6_hdr);
    	int next_header = ip6h->ip6_nxt;
	unsigned char *data = buffer + sizeof(struct ethhdr) + sizeof(struct ip6_hdr);
	while (next_header != IPPROTO_TCP && next_header != IPPROTO_NONE && next_header != IPPROTO_ICMPV6 && next_header != IPPROTO_UDP)
    	{
    	        struct ip6_ext *ext_hdr = (struct ip6_ext *)data;
    	        next_header = ext_hdr->ip6e_nxt;
    	        int ext_hdr_len = (ext_hdr->ip6e_len + 1) * 8;
    	        header_length += ext_hdr_len;
    	        data += ext_hdr_len;
    	}

	// No (more) extension headers, TCP/ICMPV6/UDP header reached
	if(next_header == IPPROTO_TCP)
    	{
    	    	struct tcphdr *tcph = (struct tcphdr *)(buffer + header_length);

		*syn_flag = tcph->syn;
		source->sin6_port = ntohs(tcph->source);
        	dest->sin6_port = ntohs(tcph->dest);
    	}


	return;
}

char* mac_to_eui64(char *const mac, char *eui64)
{
	// To lower case
	for (int i = 0; *(mac + i); ++i) mac[i] = tolower(mac[i]);
	//printf("mac in lowercase: %s", mac);

	strncpy(eui64, mac, 2);
	strncpy(eui64 + 2, mac + 3, 2);
	eui64[4] = ':';
	strncpy(eui64 + 5, mac + 6, 2);
	strncpy(eui64 + 7, "ff:fe", 5);
	strncpy(eui64 + 12, mac + 9, 2);
	eui64[14] = ':';
	strncpy(eui64 + 15, mac + 12, 2);
	strncpy(eui64 + 17, mac + 15, 2);
	eui64[19] = '\0';
	
	//Flip 7th bit
	char value;
	if(eui64[1] >= 48 && eui64[1] <= 57)
	{
		value = eui64[1] - '0';
	}
	else if (eui64[1] >= 97 && eui64[1] <= 102)
	{
		value = eui64[1] - 'a' + 10;
	}
	else
	{
		printf("%c\n", eui64[1]);
		perror("Unknown hexadecimal character within mac address\n");
		return NULL;
	}

        value ^= 0b00000010;

	if(value >= 0 && value <= 9)
        {
                value = value + '0';
        }
        else if (value >= 10 && value <= 15)
        {
                value = value - 10 + 'a';
        }

	eui64[1] = value;

	
	return eui64;
}

void derive_mesh_exit_node_MPTCP_endpoint_addresses(struct sockaddr_in6 dest, int no_endpoints, char **subflow_prefixes, char mesh_exit_node_endpoint_addresses[][INET6_ADDRSTRLEN])
{
	// Retrieve the destination address prefix
	char dst_addr_prefix[INET6_ADDRSTRLEN/2];
	ipv6_address_prefix(dest.sin6_addr, dst_addr_prefix);
	//printf("Dest IPv6 address prefix: %s\n", dst_addr_prefix);

	// Retrieve MAC address from prefix
	char dst_addr_mac[18];
	mac_from_prefix(dst_addr_prefix, dst_addr_mac);
	//printf("MAC from prefix: %s\n", dst_addr_mac);

	// Derive the suffix (eui64(mac))
	char suffix[INET6_ADDRSTRLEN/2];
	mac_to_eui64(dst_addr_mac, suffix);
	//printf("Suffix: %s\n", suffix);

	// Derive the exit mesh node endpoint addresses
	for (int subflow = 0; subflow < no_endpoints; subflow++)
	{
		strcpy(mesh_exit_node_endpoint_addresses[subflow], subflow_prefixes[subflow + 1]);
		strcat(mesh_exit_node_endpoint_addresses[subflow], ":");
		strcat(mesh_exit_node_endpoint_addresses[subflow], suffix);

		printf("Endpoint %d: %s\n", (subflow + 1), mesh_exit_node_endpoint_addresses[subflow]);
	}


	return;
}


void ipv6_address_prefix(struct in6_addr dest_ipv6_address, char *dst_addr_prefix)
{
	char dest_ipv6_address_str[INET6_ADDRSTRLEN];
	//inet_ntop(AF_INET6, &(dest_ipv6_address), dest_ipv6_address_str, INET6_ADDRSTRLEN);
	ipv6_to_str_expanded(dest_ipv6_address_str, (const struct in6_addr *)(&dest_ipv6_address));
	
	//printf("Dest IPv6 address: %s\n", dest_ipv6_address_str);

	strncpy(dst_addr_prefix, dest_ipv6_address_str, 19);
	dst_addr_prefix[19] = '\0';


	return;
}

void ipv6_to_str_expanded(char *str, const struct in6_addr *addr) {
   sprintf(str, "%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                 (int)addr->s6_addr[0], (int)addr->s6_addr[1],
                 (int)addr->s6_addr[2], (int)addr->s6_addr[3],
                 (int)addr->s6_addr[4], (int)addr->s6_addr[5],
                 (int)addr->s6_addr[6], (int)addr->s6_addr[7],
                 (int)addr->s6_addr[8], (int)addr->s6_addr[9],
                 (int)addr->s6_addr[10], (int)addr->s6_addr[11],
                 (int)addr->s6_addr[12], (int)addr->s6_addr[13],
                 (int)addr->s6_addr[14], (int)addr->s6_addr[15]);

   return;
}

void mac_from_prefix(const char *dst_addr_prefix, char *dst_addr_mac)
{
	strncpy(dst_addr_mac, dst_addr_prefix + 5, 2);
	dst_addr_mac[2] = ':';
	strncpy(dst_addr_mac + 3, dst_addr_prefix + 7, 2);
	dst_addr_mac[5] = ':';
	strncpy(dst_addr_mac + 6, dst_addr_prefix + 10, 2);
	dst_addr_mac[8] = ':';
	strncpy(dst_addr_mac + 9, dst_addr_prefix + 12, 2);
	dst_addr_mac[11] = ':';
	strncpy(dst_addr_mac + 12, dst_addr_prefix + 15, 2);
	dst_addr_mac[14] = ':';
	strncpy(dst_addr_mac + 15, dst_addr_prefix + 17, 2);
	dst_addr_mac[17] = '\0';


	return;
}


void packet_handler(u_char *user, const struct pcap_pkthdr *h, const u_char *bytes)
{
	struct sockaddr_in6 source, dest;
	bool syn_flag;

	// Size of captured bytes
    	int data_size = h->caplen;

	// Process received packet, get socket information and SYN flag value
	get_socket_info((u_char *)bytes, data_size, &source, &dest, &syn_flag);


	if(syn_flag == 1)
	{
		// Debug statements, enabled if DEBUG == 1
		#if DEBUG
			char src_addr[INET6_ADDRSTRLEN];
			char dst_addr[INET6_ADDRSTRLEN];
			printf("SYN flag: %d\n", syn_flag);
			printf("Source IP: %s\n", inet_ntop(AF_INET6, &(source.sin6_addr), src_addr, INET6_ADDRSTRLEN));
			printf("Dest IP: %s\n", inet_ntop(AF_INET6, &(dest.sin6_addr), dst_addr, INET6_ADDRSTRLEN));
        		printf("Source. port: %d\n", source.sin6_port);
        		printf("Dest. port: %d\n", dest.sin6_port);
		#endif

		struct argc_argv *argc_argv_data = (struct argc_argv *)user;
        	char mesh_exit_node_endpoint_addresses[argc_argv_data->argc - 1][INET6_ADDRSTRLEN];
		
		int argc = argc_argv_data->argc;
		char **argv = argc_argv_data->argv;

		// Derive exit mesh node endpoint addresses
		derive_mesh_exit_node_MPTCP_endpoint_addresses(dest, argc - 1, argv, mesh_exit_node_endpoint_addresses);

		// Execute the proxy client
		fp = fopen("ss-redir_mptcp_slaac.json", "w");
		fprintf(fp,
				 "{\
				 \n\t\"server\" : ["
		       );

		for (int subflow = 0; subflow < argc - 1; subflow++)
		{
		        if(subflow != argc - 2)
		        		fprintf(fp, "\"%s\", ", mesh_exit_node_endpoint_addresses[subflow]);
		        else
		        		fprintf(fp, "\"%s\"", mesh_exit_node_endpoint_addresses[subflow]);
		}

		fprintf(fp,
				 "],\
				 \n\t\"server_port\" : 8388,\
				 \n\t\"local_address\" : \"::0\",\
				 \n\t\"local_port\" : 1080,\
				 \n\t\"password\" : \"sai\",\
				 \n\t\"timeout\" : 300,\
				 \n\t\"method\" : \"aes-256-cfb\"\
				 \n}\
				 \n"
		       );
		fflush(fp);
		fclose(fp);
		system("mptcpize run ss-redir -c ss-redir_mptcp_slaac.json &");


		// Unblock all TCP packets from connected end-host
       		fp = fopen("iptablerules_unblock_mptcp_slaac.sh", "w");
       		fprintf(fp,
       		                 "ip6tables -t filter -D INPUT -p tcp -s fdff::0/16 -j DROP"
       		       );
       		fflush(fp);
       		fclose(fp);
       		system("bash iptablerules_unblock_mptcp_slaac.sh");

       		exit(0); // Exit program
	}
}
