FROM ubuntu:18.04

RUN apt-get update -y && apt-get upgrade -y

RUN apt-get install -y coturn

# might need -L for IP Addr: https://meetrix.io/blog/webrtc/turnserver/long_term_cred.html
CMD turnserver -a -f --use-auth-secret --use-auth-secret --static-auth-secret=$COTURN_SECRET --min-port 10000 --max-port 10100 --realm=$SITE_ADDR -v --external-ip=lilflo.com --cert=/etc/letsencrypt/live/lilflo.com/fullchain.pem --pkey=/etc/letsencrypt/live/lilflo.com/privkey.pem
