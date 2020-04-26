FROM Ubuntu:16.04

RUN apt-get update -y && apt-get upgde -y

RUN apt-get install -y coturn

# might need -L for IP Addr: https://meetrix.io/blog/webrtc/turnserver/long_term_cred.html
CMD ["turnserver -a -f --use-auth-secret --static-auth-secret=$COTURN_SECRET --min-port 10000 --max-port 10100 -p 3478 --realm=$SITE_ADDR"]
