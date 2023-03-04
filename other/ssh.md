---
description: How to Enable SSH on Ubuntu 22.04
---

# SSH

```bash
sudo apt install openssh-server -y  # install ssh server

sudo systemctl status ssh     # se the systemctl command to check its status
```

The SSH service is in the running state, now we will allow the connection on the SSH port by using the ufw command:

```bash
sudo ufw allow ssh
```

### How to connect the computer by using the SSH

To connect any other system through the SSH, first make sure both machines have SSH server installed and enabled. Secondly, you should know the IP address and user name of the machine you want to connect;

```bash
ssh <name>@<address>
```
