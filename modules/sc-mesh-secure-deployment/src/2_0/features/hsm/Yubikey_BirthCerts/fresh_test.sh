#! /bin/bash

echo -e "\033[32;1m"
figlet -t `date`
echo -e "\033[0m"
rm -rf ./crypto; bash -ex 00_boot.sh
