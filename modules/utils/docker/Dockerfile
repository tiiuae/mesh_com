FROM comms_vm
COPY entrypoint.sh /usr/local/bin/
COPY entrypoint_nats.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint_nats.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint_nats.sh"]
