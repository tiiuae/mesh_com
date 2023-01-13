FROM comms_vm
COPY entrypoint.sh /usr/local/bin/
COPY mesh.conf /opt
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]