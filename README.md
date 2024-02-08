# nRF-Sensor-Django-Project
- Some stuff to look into to do in my own time:
    - [Zephry Over-the-Air](https://docs.zephyrproject.org/latest/services/device_mgmt/ota.html)

## What you need
Open miniforge and if you have not installed the following libraries in your conda environment then do the following:
```bash
mamba install django channels daphne
```

We are going to install a **Redis Python Client**, where we use the following [link](https://github.com/redis/redis-py) and do:
```bash
mamba install redis-py
```


# Creating a Docker Redis Container


Open Miniforge Prompt, where we are going to create and name a **Redis Docker Container**:
```bash
docker run --name my_redis -d redis
```

Now that we created a **Redis Docker Container** and a random ID generator should be given to you. Where that is the ID for the container and it is highly recommended to save the ID where it will be used for the python portion of the code. 

To verify that we created the **Redis Docker Container** we are going to use the command line:
```bash
docker ps
```
Where the command line gives us the directory of the current paths or containers that are made.

*IF YOU HAVE MORE THAN ONE DOCKER OPEN, YOU NEED TO CLOSE EVERYTHING BUT my_redis.* You do that by doing the following:
```bash
docker stop docker_file_name
```

To check if the extra docker actually closed, we do 
```bash
docker ps
```

## Checking and Running a Redis Docker Container
Lets say we have a bunch of containers and each time we turn on the computer. We need to run the container to make it work again rather than creating a new one. So we do the following:
```bash
docker container ls
```
This command will list all the global command line tell us how many services/containers are actually open. However, we want to run an existing container that we created, so we look do the following:
```bash 
docker run --name my_redis
```

To verify that the container is running we are going to use the following command line:
```bash 
docker images
```
Where you can see the current containers that is being run!!!

## Allowing Docker to have public access to a container (for the port)

You must completed the previos task or have **my_redis** running at least. Where we need to tell docker, we want to virtually give people have access to the the container like sending or receiving information from the container or client. Where we do the following:
```bash
docker run --name my_redis -p 6379:6379 -d redis
```
6379 is the port of the address


**NOTE FOR SOME REASON IT DOES NOT ALLOW YOU TO CREATE THE PORT, THEN JUST DO:**
```bash
docker rm "the_docker_id_tag"
```
**Again, only do this if, you cant do the first command line!!!**

Once you are able to create/run the docker container, we need to check the ADDRESS and PORT that the docker allows the client to use *(our case, its localhost:6379)*. We check by doing the following:
```bash
docker ps
```
You are done!!! Just have the **Python Redis Client Code Ready:0**

# Starting a Redis Docker Python Client (Initializing the container)

To start running a **Redis Docker**, you do the following:
```bash
docker start my_redis -a
```
Where *-a* means immediately

Now you need to check if it works on your computer, the python file asking for a request.

## Redis-py Client

We are going to use a **Redis Python Client**, if you do not have it then install it using miniforge. The following format to connect to a Redis Docker Container we need to use in python:
```python
>>> import redis

# You will need an IP ADDRESS, the PORT of the address, and db is decode_response (true or false using 1 or 0)
>>> r = redis.Redis(host='localhost', port=6379, db=0)

>>> r.set('foo', 'bar')
True

>>> r.get('foo')
b'bar'
```
**VERY IMPORTANT AND MUST READ**, you need to check what server is your IP ADDRESS and PORT you are using
