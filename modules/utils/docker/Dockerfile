FROM comms_vm
COPY entrypoint.sh /usr/local/bin/
COPY entrypoint_ms.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint_ms.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
