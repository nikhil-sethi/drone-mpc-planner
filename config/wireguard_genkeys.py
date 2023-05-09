#!/usr/bin/env python3

import os
import wgconfig
import wgconfig.wgexec as wgexec
from tqdm import tqdm


folder = os.path.expanduser('~/wireguard_config/')
if not os.path.exists(folder):
    os.mkdir(folder)
wg_conf_file = folder + 'wg0.conf'
wc = wgconfig.WGConfig(wg_conf_file)
if os.path.exists(wg_conf_file):
    test = wc.read_file()
# print('INTERFACE DATA:', wc.interface)
# print('PEER DATA (there are no peers yet):', wc.peers)

port = 51826
start_range = '10.13.'
server_range = 9
wireguard_server_ip = start_range + str(server_range) + '.1'
n_peers = 1000  # from zero


def create_peer(wc: wgconfig.WGConfig, peer_name: str, ip3: int, ip4: int, vip: bool) -> None:
    public_key_server = wgexec.get_publickey(wc.interface['PrivateKey'])
    private_key, public_key = wgexec.generate_keypair()
    preshared_key = wgexec.generate_presharedkey()
    wc.add_peer(public_key, '# ' + peer_name)
    wc.add_attr(public_key, 'PresharedKey', preshared_key)
    wc.add_attr(public_key, 'AllowedIPs', start_range + str(ip3) + '.' + str(ip4) + '/32', append_as_line=True)

    if not os.path.exists(folder + 'peers/'):
        os.mkdir(folder + 'peers/')
    peer_conf_file = folder + 'peers/' + peer_name + '.conf'
    wc_peer = wgconfig.WGConfig(peer_conf_file)
    wc_peer.initialize_file()

    wc_peer.add_attr(None, 'Address', start_range + str(ip3) + '.' + str(ip4))
    wc_peer.add_attr(None, 'PrivateKey', private_key)
    wc_peer.add_attr(None, 'ListenPort', port)
    # wc_peer.add_attr(None, 'DNS', wireguard_server_ip)
    wc_peer.add_attr(None, 'PostUp', 'ping -c1 ' + wireguard_server_ip)

    wc_peer.add_peer(public_key_server)
    wc_peer.add_attr(public_key_server, 'PresharedKey', preshared_key, append_as_line=True)
    wc_peer.add_attr(public_key_server, 'Endpoint', 'vpn.pats-c.com:' + str(port), append_as_line=True)
    wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range + str(server_range) + '.0/24', append_as_line=True)
    if vip:
        wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range + str(server_range + 1) + '.0/24', append_as_line=True)
    wc_peer.add_attr(public_key_server, 'PersistentKeepalive', '25', append_as_line=True)
    wc_peer.write_file()


def update_peer(wc: wgconfig.WGConfig, peer_name: str, ip3: int, ip4: int, vip: bool) -> None:
    peer_conf_file = folder + 'peers/' + peer_name + '.conf'
    public_key_server = wgexec.get_publickey(wc.interface['PrivateKey'])
    wc_peer = wgconfig.WGConfig(peer_conf_file)
    wc_peer.read_file()
    private_key = wc_peer.interface['PrivateKey']
    preshared_key = wc_peer.peers[public_key_server]['PresharedKey']
    wc_peer.initialize_file()

    wc_peer.add_attr(None, 'Address', start_range + str(ip3) + '.' + str(ip4))
    wc_peer.add_attr(None, 'PrivateKey', private_key)
    wc_peer.add_attr(None, 'ListenPort', port)
    # wc_peer.add_attr(None, 'DNS', wireguard_server_ip)
    wc_peer.add_attr(None, 'PostUp', 'ping -c1 ' + wireguard_server_ip)

    wc_peer.add_peer(public_key_server)
    wc_peer.add_attr(public_key_server, 'PresharedKey', preshared_key, append_as_line=True)
    wc_peer.add_attr(public_key_server, 'Endpoint', 'vpn.pats-c.com:' + str(port), append_as_line=True)
    wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range + str(server_range) + '.0/24', append_as_line=True)
    if vip:
        for i in range(1,11):
            wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range + str(server_range + i) + '.0/24', append_as_line=True)
        
    wc_peer.add_attr(public_key_server, 'PersistentKeepalive', '25', append_as_line=True)
    wc_peer.write_file()


vips = ['dash', 'beta', 'dummy1', 'dummy1', 'kevin', 'sjoerd', 'wouter_o','wouter_vdh', 'jorn', 'rik','lotte','dayo','frank']
if len(wc.peers):
    for i, vip in enumerate(vips):
        if os.path.exists(folder + 'peers/' + vip + '.conf'):
            update_peer(wc, vip, server_range, i + 2, True)
        else:
            create_peer(wc, vip, server_range, i + 2, True)


    pbar = tqdm(range(1, len(wc.peers) - len(vips)))
    for i in pbar:
        ip4 = i % 255
        ip3 = int((i - ip4) / 255 + server_range + 1)
        update_peer(wc, 'pats' + str(i), ip3, ip4, False)
    pbar = tqdm(range(len(wc.peers) - len(vips), n_peers))
    for i in pbar:
        ip4 = i % 255
        ip3 = int((i - ip4) / 255 + server_range + 1)
        create_peer(wc, 'pats' + str(i), ip3, ip4, False)
else:
    wc.initialize_file()
    wc.add_attr(None, 'Address', wireguard_server_ip)
    wc.add_attr(None, 'ListenPort', port)
    wc.add_attr(None, 'PrivateKey', wgexec.generate_privatekey())
    wc.add_attr(None, 'PostUp', 'iptables -A FORWARD -i %i -j ACCEPT; iptables -A FORWARD -o %i -j ACCEPT; iptables -t nat -A POSTROUTING -o eth+ -j MASQUERADE')
    wc.add_attr(None, 'PostDown', 'iptables -D FORWARD -i %i -j ACCEPT; iptables -D FORWARD -o %i -j ACCEPT; iptables -t nat -D POSTROUTING -o eth+ -j MASQUERADE')

    for i, vip in enumerate(vips):
        create_peer(wc, vip, server_range, i + 2, True)
    # when adding a server, don't forget to adjust the -2 below

pbar = tqdm(range(len(wc.peers) - len(vips), n_peers))
for i in pbar:
    ip4 = i % 255
    ip3 = int((i - ip4) / 255 + server_range + 1)
    create_peer(wc, 'pats' + str(i), ip3, ip4, False)

wc.write_file()
