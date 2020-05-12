# Access SFTP with WinSCP

## Requirements:
* .ppk file with keys

## Connecting
1. In WinSCP go to advance settings.
![advance settings](https://github.com/pats-drones/pats/blob/master/doc/advance_settings.png)
2. In __Directories__ enter Remote directory and Local directory.
![advance settings](https://github.com/pats-drones/pats/blob/master/doc/loc_dir.png)
3. In __Tunnel__ enter hostname, port number, username and .ppk file containing private key.
![advance settings](https://github.com/pats-drones/pats/blob/master/doc/tunnel.png)
4. In __Authentication__ add .ppk file containing private key.
![advance settings](https://github.com/pats-drones/pats/blob/master/doc/auth.png)

You are now able to connect to the SFTP.