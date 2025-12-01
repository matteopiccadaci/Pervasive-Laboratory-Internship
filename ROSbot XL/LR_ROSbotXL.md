# Lightning-Rod 0.5.0 on Rosbot XL
The new version of **Lightning-Rod (0.5.0)'s installation procedure on Rosbot XL requires some modification in order to make it work properly. Some of the operations are similar to the ones found for the previous version (<a href="https://opendev.org/x/iotronic-lightning-rod/src/branch/master/doc/installation/raspberry_pi_3.rst">Old version</a>): here follow the additional steps needed to make it work.

## Prerequisites and Installation
```
sudo apt install python3 python3-setuptools python3-pip gdb lsof libssl-dev libffi-dev
```
```
pip3 install pyasyncore
```
```
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs
sudo apt install -y npm
sudo npm install -g npm
echo "NODE_PATH=/usr/lib/node_modules" | sudo tee -a /etc/environment
source /etc/environment > /dev/null
``` 
```
sudo npm install -g --unsafe @mdslab/wstun
```
```
sudo apt install -y nginx
sudo sed -i 's/# server_names_hash_bucket_size 64;/server_names_hash_bucket_size 64;/g' /etc/nginx/nginx.conf
sudo sed -i "s|listen 80 default_server;|listen 50000 default_server;|g" /etc/nginx/sites-available/default
sudo sed -i "s|80 default_server;|50000 default_server;|g" /etc/nginx/sites-available/default
```
```
pip3 install certbot certbot-nginx
```

```
pip3 install iotronic-lightningrod
```

Since Lighting-Rod is going to be installed by the *husarion* user (in order to make the ROS2 integration work), it is necessary to create the following folders and set the correct permissions:
```
# ========== /etc/iotronic ==========
sudo mkdir -p /etc/iotronic
sudo touch /etc/iotronic/settings.json
sudo touch /etc/iotronic/iotronic.conf
sudo chown -R husarion:husarion /etc/iotronic
sudo chmod 755 /etc/iotronic

# ========== /var/lib/iotronic ==========
sudo mkdir -p /var/lib/iotronic/plugins
sudo touch /var/lib/iotronic/plugins.json
sudo touch /var/lib/iotronic/services.json
sudo chown -R husarion:husarion /var/lib/iotronic
sudo chmod 755 /var/lib/iotronic
sudo chmod 755 /var/lib/iotronic/plugins

# ========== /var/log/iotronic ==========
sudo mkdir -p /var/log/iotronic
sudo chown husarion:husarion /var/log/iotronic
sudo chmod 755 /var/log/iotronic

# ========== logrotate ==========
sudo touch /etc/logrotate.d/lightning-rod.log
sudo chown husarion:husarion /etc/logrotate.d/lightning-rod.log

# ========== systemd service ==========
sudo touch /etc/systemd/system/lightning-rod.service
sudo chown husarion:husarion /etc/systemd/system/lightning-rod.service

```

```
lr_install
```
```
systemctl start lightning-rod.service
```
```
tail -f /var/log/iotronic/lightning-rod.log
```
This part is rather similar to the Old version with just few adjustments due to, for example, the new certbot package name.

## Post Installation
### Crossbar Certificate
In order to be sure that the communication between the boards and *Crossbar* work, it is necessary to copy its certificate on the boards on which **Lightning-Rod** will run. To do so, follow these steps:

- After starting the Docker compose wait for the *ca-service* container to finish its routine. After that go to:
    ```
    /etc/ssl/iotronic
    ```
    and copy the **crossbar.pem** file to your *LR* machine using, for example, *scp*.
&nbsp;

- On the *LR* machine it's necessary to add the certificate to the trusted ones. To do so, copy the **crossbar.pem** file to:
    ```
    sudo cp /tmp/crossbar.pem /usr/local/share/ca-certificates/crossbar.crt
    ```
    and then issue the command:
    ```
    update-ca-certificates
    ```
    You should see something like:
    ```
    Updating certificates in /etc/ssl/certs...
    1 added, 0 removed; done.
    Running hooks in /etc/ca-certificates/update.d...
    done.
    ```
&nbsp;
To verify that the certificate has been correctly added, you can perform a *curl* command towards the **Crossbar** server:
```
curl -v https://<WAMP-SERVER>:<WAMP-PORT>/
```
or try to visit the page using a browser. 
### Lightning-Rod
After the first installation, I discovered that the Web configuration does not work properly. In order to make the Raspberry correctly connect to Stack4Things, that's the list of operation neeeded:

- Add the Rosbot XL on Stack4Things (if not done yet) and copy the Board Name.
&nbsp;

- Issue the command:
    ```
    lr_install
    ```
    Then start and immediately stop the service:
    ```
    systemctl start lightning-rod.service
    systemctl stop lightning-rod.service
    ```
&nbsp;
- Edit the configuration file:

    ```
    /etc/iotronic/settings.json
    ```
    and modify it so that it looks like this:
    ```
    {
    "iotronic": {
        "board": {
        "code": "<BOARD-NAME>",
        "hostname": "<BOARD-NAME>"
        },
        "wamp": {
        "registration-agent": {
            "url": "wss://<WAMP-SERVER>:<WAMP-PORT>/",
            "realm": "s4t"
        }
        }
    }
    }
    ```
    Of course, the information about the WAMP server must be filled in with the values of the machine on which S4T runs.

    &nbsp;

- Start the service again, wait a couple of minutes and stop it again:
    ```
    sudo lightning-rod
    systemctl stop lightning-rod.service
    ```
    Starting the service will trigger a modification to the *settings.json* file, which will look like this:

    ```
    {
        "iotronic": {
            "extra": {},
            "board": {
                "id": 1,
                "uuid": "ad5107bb-c6a7-4a99-8ee5-087b6869bb71",
                "code": "<BOARD-NAME>",
                "status": "registered",
                "name": "<BOARD-NAME>",
                "type": "gateway",
                "agent": "iotronic-wagent",
                "owner": "d0c41fe20cfc4ee1954f27c21dd0fec8",
                "project": "82b9d75d135f4476a7aacad4985b6a53",
                "fleet": null,
                "lr_version": null,
                "connectivity": {},
                "mobile": false,
                "extra": {},
                "created_at": "2025-10-09T11:04:36+00:00",
                "updated_at": null,
                "location": {
                    "longitude": "0.0",
                    "latitude": "0.0",
                    "altitude": "0.0",
                    "updated_at": "2025-10-09T11:04:36+00:00"
                }
            },
            "wamp": {
                "registration-agent": {
                    "url": "wss://crossbar:8181/",
                    "realm": "s4t"
                },
                "main-agent": {
                    "url": "wss://crossbar:8181/",
                    "realm": "s4t"
                }
            }
        }
    }
    ```
    The url is automatically set to *wss://crossbar:8181/*, which is not correct. So, edit the file again and set the correct url of the WAMP server.
    The previous step is optional: you can immediately use the correct *settings.json* file with the above structure. I suggest not to do so because of parameteres like *owner*, *project* and *id* that are automatically set.
&nbsp;
- Start the service again and now everything should be ready:
    ```
    systemctl start lightning-rod.service
    ```
    &nbsp;

You can check the log file to see if everything is working properly:
```
tail -f /var/log/iotronic/lightning-rod.log
```
If you see as last line:
```
Listening...
```
The installation of **Lightning-Rod 0.5.0** on Rosbot XL is complete and the robot should be visible on Stack4Things.
