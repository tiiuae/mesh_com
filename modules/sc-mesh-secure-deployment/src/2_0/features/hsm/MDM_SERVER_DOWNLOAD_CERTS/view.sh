#! /bin/sh

openssl x509 -noout -text -in "$1" | less
