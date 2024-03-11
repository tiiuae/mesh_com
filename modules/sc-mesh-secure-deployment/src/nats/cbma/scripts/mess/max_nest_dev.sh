#! /bin/bash
#
# This script creates a dummy/batadv/bridge/batadv/bridge/etc stack as tall as
# possible to detect the MAX_NEST_DEV compiled in the currently running kernel
#
# https://github.com/torvalds/linux/blob/master/include/linux/netdevice.h#L96
#
# Increasing MAX_NEST_DEV from the 8 default to at least 12 is necessary to be
# able to instantiate the Mesh Shield 2.0 stack of virtual devices looking like
# ...: VLANs or other SDN stuff the customer might want
# br-red: Red Bridge formerly known as br-lan
# bat1: Upper batman-adv to be given a better name
# umb002091a3b4c5: Upper MAC Bridge transmitting from our 00:20:91:a3:b4:c5 MAC
# ums002091f8e7d6: Upper MACsec decrypting what 00:20:91:f8:e7:d6 encrypted
# umv002091f8e7d6: Upper MACVLAN receiving only from 00:20:91:f8:e7:d6 peer
# br-white: White Bridge possibly useful only for MDM Server
# bat0: Lower batman-adv to be given a better name
# lmb00301af5e4d3: Lower MAC Bridge for our 00:30:1a:f5:e4:d3 Doodle Labs card
# lms04f021a6b7c8: Lower MACsec decrypting what 04:f0:21:a6:b7:c8 encrypted
# lmv04f021a6b7c8: Lower MACVLAN receiving only from 04:f0:21:a6:b7:c8 Compex
# wlp1s0: our Doodle Labs card with a 00:30:1a:f5:e4:d3 MAC address



DEV_PREFIX='max_nest_dev'



recursively_stack_interfaces()
{
	local DEV_COUNT="$1"

	local THIS_DEV="${DEV_PREFIX}$DEV_COUNT"
	local PREV_DEV="${DEV_PREFIX}$(( $DEV_COUNT - 1 ))"
	if [ 0 -ne $(( $DEV_COUNT % 2 )) ]; then
		TYPE='batadv'
	else
		TYPE='bridge'
	fi
	if ip link add name "$THIS_DEV" type "$TYPE"; then
		if STDOUTERR=`ip link set dev "$PREV_DEV" master "$THIS_DEV" 2>&1`; then
			recursively_stack_interfaces $(( $DEV_COUNT + 1 ))
			ip link set dev "$PREV_DEV" nomaster
		else
			if [ "$STDOUTERR" != 'RTNETLINK answers: Too many links' ]; then
				>&2 echo "Unexpected: $STDOUTERR"
			fi
			echo "$DEV_COUNT"
			if [ $DEV_COUNT -lt 11 ]; then
				>&2 echo 'Warning: you need MAX_NEST_DEV >= 11 to run the full MS2 stack'
			fi
		fi
		ip link delete "$THIS_DEV"
	else
		>&2 echo 'Warning: this should not happen!'
	fi
}



if ip link add name "${DEV_PREFIX}0" type dummy; then
	recursively_stack_interfaces 1
	ip link delete "${DEV_PREFIX}0"
else
	>&2 echo 'Cannot run at all!'
	exit 1
fi
