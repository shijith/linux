#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
#
# Run a couple of IP defragmentation tests.

set +x
set -e

readonly NETNS="ns-$(mktemp -u XXXXXX)"

setup() {
	ip netns add "${NETNS}"
	ip -netns "${NETNS}" link set lo up
	ip netns exec "${NETNS}" sysctl -w net.ipv4.ipfrag_high_thresh=9000000 >/dev/null 2>&1
	ip netns exec "${NETNS}" sysctl -w net.ipv4.ipfrag_low_thresh=7000000 >/dev/null 2>&1
	ip netns exec "${NETNS}" sysctl -w net.ipv6.ip6frag_high_thresh=9000000 >/dev/null 2>&1
	ip netns exec "${NETNS}" sysctl -w net.ipv6.ip6frag_low_thresh=7000000 >/dev/null 2>&1
}

cleanup() {
	ip netns del "${NETNS}"
}

trap cleanup EXIT
setup

echo "ipv4 defrag"
ip netns exec "${NETNS}" ./ip_defrag -4


echo "ipv4 defrag with overlaps"
ip netns exec "${NETNS}" ./ip_defrag -4o

echo "ipv6 defrag"
ip netns exec "${NETNS}" ./ip_defrag -6

echo "ipv6 defrag with overlaps"
ip netns exec "${NETNS}" ./ip_defrag -6o

