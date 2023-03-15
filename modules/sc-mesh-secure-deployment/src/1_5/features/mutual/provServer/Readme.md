# Provisioning Server for Mutual Authentication

## How to use
The whole configuration and provisioning system comprises two discrete components from the functionality perspective; An admin console and the config-service. 

The admin console is the placeholder/server from where all the device configuration parameters can be configured for any incoming client connection. It is to be noted that the admin console host machine and the incoming device request (where the config-service would be running) should be connected over the same access point. 

Installing the configuration and provisioning service:
 
Clone the repository: `https://github.com/tiiuae/configurationservice.git`   on the host machine

 

### Windows OS
If Host machine/device:   Used as an Admin Console:

- Install MySQL and create a schema named: admin-console

- Install Spring Boot | IntelliJ IDEA (jetbrains.com)

- Install Node.js

- Restart the system

- npm install -g @angular/cli@10.0.0

- run mvn package

- ng serve: After successful execution, the admin console UI should appear at localhost:420

### LINUX OS
On the Linux os, some additional stuff needs to be done. Please follow the below stuff to get going

1. Clone the GitHub Repository


`sudo git clone https://github.com/tiiuae/configurationservice.git`
 

2. Go into configuratiionservice>admin-console-ui>

   - Follow the below steps to Install Node.js

````
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | bash
source ~/.bashrc
nvm list-remote
nvm install v16.16.0
````

3.  Do sudo su


`npm install -g @angular/cli@10.0.0 --force`
 

4. Go to configuratiionservice>Admin-Console


`sudo mvn package`
 

5. Again configuratiionservice>admin-console-ui>


`npm start`
 

 

Common Steps after the above steps:

 

* Open the AdminConsoleApplication present in the folder Admin-    Console>src>main>java>com.adminconsole in IntelliJ IDEA.

* At the top select Edit Configurations and do the following: 1) Click on Modify options and enable “Add VM options”, 2) For the option Build and run select: java 18 openjdk-18 option in the first column and paste the “-Dspring.profiles.active=dev” in the second column. For the below column put  “com.adminconsole.AdminConsoleApplication”. Do the apply.

* Go to Admin-    Console>src>main>java>resources in IntelliJ IDEA and Update the root user / PWD in the properties: application.properties and application-dev.properties for the SQL schema username and password  (lines 13 and 14 and lines 8 and 9 in the scripts, respectively)

* Now go to the admin-console-ui folder>src>environments: In the file environment.ts in line 7 change https to http.

* Run the AdminConsoleApplication in IntelliJ IDE

* After execution, go to the MySQL and under the admin-console schema in the Table>role. Create a role manually with id “1” and name” ADMIN: and do apply.

* Go to the web browser and open the link `http://localhost:4200/`. Register the user, if not. Login with the credentials if already registered.

 

#### If Host machine/device:   Used for config-service
 

Same steps on both Windows and Linux

 

Do an SCP to transfer the configuration service to the comm sleeve devices.

First of all 

1. Go to the /configurationservice/config-service directory

2. To create a docker image:


```docker build -t config_service .```
 (use sudo if not using on comm sleeve)

3. After successful image building: 


```docker run -d --network host config_service``` 

Check whether the container is UP and copy the container id: 


``docker ps -a``

Executing the config_service: 


`docker exec -it  <container id>  sh`
 

##### If the JSON file has to be viewed outside the container

```
docker run -d --network host --volume config_service path:/root/ config_service 
docker exec -it  <container id>  cp /config.json /root/
```

The above steps complete the execution at the config_service. Now go to the server webpage, to check for the device. If the device ip address does not appear on the list--- ping the device from the server

After Ping, the device will be visible in the list. On the menu set all the desired parameters, save the root certificate and press APPLY

The APPLY will trigger a config.json file which could be seen on the client device. The JSON file contains all the required parameters. 

If any parameters for the device need to be changed, just go to the admin console UI and change/add the parameters and do an APPLY.

 

Note: After every fresh running remove all the stopped containers via docker systems prune -a