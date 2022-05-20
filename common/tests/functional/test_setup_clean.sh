#!/bin/bash

source ./common/common.sh   # common tools
source ./common/init.sh     # network init
source ./common/deinit.sh   # network de-init

#######################################
# main
# Globals:
#  result
# Arguments:
#######################################
main() {
  echo "Cleans commonly used setup in Functional Test cases." | print_log result
  _deinit
  echo "Done." | print_log result
}

main "$@"