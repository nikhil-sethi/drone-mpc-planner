#!/usr/bin/env python3

import os
import wgconfig
import wgconfig.wgexec as wgexec
from tqdm import tqdm


folder = os.path.expanduser('~/wireguard_config/')
if not os.path.exists(folder):
    os.mkdir(folder)
wg0_conf_file = folder + 'wg0.conf'
wc_wg0_ori = wgconfig.WGConfig(wg0_conf_file)
if os.path.exists(wg0_conf_file):
    wc_wg0_ori.read_file()

port = 51826
mtu = 1200
start_range = '10.13.'
server_range = 9
wireguard_server_ip = start_range + str(server_range) + '.1'
n_peers = 1000  # from zero

def create_peer(wc_wg0: wgconfig.WGConfig, peer_name: str, ip3: int, ip4: int, vip: bool) -> None:
    public_key_server = wgexec.get_publickey(wc_wg0.interface['PrivateKey'])
    private_key, public_key = wgexec.generate_keypair()
    preshared_key = wgexec.generate_presharedkey()
    wc_wg0.add_peer(public_key, '# ' + peer_name)
    wc_wg0.add_attr(public_key, 'PresharedKey', preshared_key)
    wc_wg0.add_attr(public_key, 'AllowedIPs', start_range + str(ip3) + '.' + str(ip4) + '/32', append_as_line=True)

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
    wc_peer.add_attr(None, 'MTU', mtu)

    wc_peer.add_peer(public_key_server)
    wc_peer.add_attr(public_key_server, 'PresharedKey', preshared_key, append_as_line=True)
    wc_peer.add_attr(public_key_server, 'Endpoint', 'vpn.pats-c.com:' + str(port), append_as_line=True)
    if vip:
        wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range  + '0.0/19', append_as_line=True)
    else:
        wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range + str(server_range) + '.0/24', append_as_line=True)
    wc_peer.add_attr(public_key_server, 'PersistentKeepalive', '25', append_as_line=True)
    wc_peer.write_file()


def update_peer(wc_wg0: wgconfig.WGConfig, peer_name: str, ip3: int, ip4: int, vip: bool) -> None:
    peer_conf_file = folder + 'peers/' + peer_name + '.conf'
    public_key_server = wgexec.get_publickey(wc_wg0.interface['PrivateKey'])
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
    wc_peer.add_attr(None, 'MTU', mtu)

    wc_peer.add_peer(public_key_server)
    wc_peer.add_attr(public_key_server, 'PresharedKey', preshared_key, append_as_line=True)
    wc_peer.add_attr(public_key_server, 'Endpoint', 'vpn.pats-c.com:' + str(port), append_as_line=True)
    
    if vip:
        wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range  + '0.0/19', append_as_line=True)
    else:
        wc_peer.add_attr(public_key_server, 'AllowedIPs', start_range + str(server_range) + '.0/24', append_as_line=True)    

    wc_peer.add_attr(public_key_server, 'PersistentKeepalive', '25', append_as_line=True)
    wc_peer.write_file()


vips = ['dash', 'beta', 'server_dummy1', 'server_dummy2', 'kevin', 'sjoerd', 'wouter_o', 'wouter_vdh', 'jorn', 'rik','lotte','dayo','frank', 'daniel', 'patser_dummy1', 'patser_dummy2', 'patser_dummy3', 'patser_dummy4', 'patser_dummy5', 'patser_dummy6', 'patser_dummy7', 'patser_dummy8', 'patser_dummy9', 'patser_dummy10', 'patser_dummy11']
vips_with_existing_keys = {}
for key,peer in wc_wg0_ori.peers.items():
    if '# pats' in peer['_rawdata'][0] and peer['_rawdata'][0][6:].isnumeric():
        continue
    peer_is_vip = False
    for vip in vips:
        if '# ' + vip == peer['_rawdata'][0]:
            vips_with_existing_keys[vip] = key
            peer_is_vip = True
            break
    if not peer_is_vip:
        wc_wg0_ori.del_peer(key) 
        old_conf_path = folder + 'peers/' + peer['_rawdata'][0][2:] + '.conf'
        if os.path.exists(old_conf_path):
            os.remove(old_conf_path)

if len(wc_wg0_ori.peers):
    for i, vip in enumerate(vips):
        if vip in vips_with_existing_keys:
            update_peer(wc_wg0_ori, vip, server_range, i + 2, True)
        else:
            create_peer(wc_wg0_ori, vip, server_range, i + 2, True)
            print('Created new vip: ' + vip)

    pbar = tqdm(range(1, len(wc_wg0_ori.peers) - len(vips)+1))
    for i in pbar:
        ip4 = i % 255
        ip3 = int((i - ip4) / 255 + server_range + 1)
        update_peer(wc_wg0_ori, 'pats' + str(i), ip3, ip4, False)
    pbar = tqdm(range(len(wc_wg0_ori.peers) - len(vips)+1, n_peers-1))
    for i in pbar:
        ip4 = i % 255
        ip3 = int((i - ip4) / 255 + server_range + 1)
        create_peer(wc_wg0_ori, 'pats' + str(i), ip3, ip4, False)
else:
    wc_wg0_ori.initialize_file()
    wc_wg0_ori.add_attr(None, 'Address', wireguard_server_ip)
    wc_wg0_ori.add_attr(None, 'ListenPort', port)
    wc_wg0_ori.add_attr(None, 'PrivateKey', wgexec.generate_privatekey())
    wc_wg0_ori.add_attr(None, 'PostUp', 'iptables -A FORWARD -i %i -j ACCEPT; iptables -A FORWARD -o %i -j ACCEPT; iptables -t nat -A POSTROUTING -o eth+ -j MASQUERADE')
    wc_wg0_ori.add_attr(None, 'PostDown', 'iptables -D FORWARD -i %i -j ACCEPT; iptables -D FORWARD -o %i -j ACCEPT; iptables -t nat -D POSTROUTING -o eth+ -j MASQUERADE')

    for i, vip in enumerate(vips):
        create_peer(wc_wg0_ori, vip, server_range, i + 2, True)
    # when adding a server, don't forget to adjust the -2 below

    pbar = tqdm(range(len(wc_wg0_ori.peers) - len(vips)+1, n_peers-1))
    for i in pbar:
        ip4 = i % 255
        ip3 = int((i - ip4) / 255 + server_range + 1)
        create_peer(wc_wg0_ori, 'pats' + str(i), ip3, ip4, False)

wc_wg0_ori.write_file()
