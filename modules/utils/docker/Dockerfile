FROM comms_vm
COPY entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
COPY olsrd /usr/local/bin/
RUN chmod +x /usr/local/bin/olsrd
CMD [&quot;/usr/local/bin/olsrd&quot;]
COPY olsrd.conf /etc/olsrd/]
