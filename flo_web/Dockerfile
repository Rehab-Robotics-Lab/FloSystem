#NOTE: This needs to be built from the LilFloSystem directory.
#So you can run `docker build -f flo_web/Dockerfile .`

# Stage 0, "build-stage", based on Node.js, to build and compile the frontend
FROM node:12 as build-web-app
WORKDIR /web_app
COPY flo_web/web_app/package*.json /web_app/
RUN npm install

COPY ./flo_web/web_app/ /web_app/

# Copy in the webrtc files:
WORKDIR /git
RUN git clone https://github.com/RobotWebTools/webrtc_ros.git
RUN rm /web_app/public/web
RUN cp -r webrtc_ros/webrtc_ros/web /web_app/public/
WORKDIR /web_app

# Copy in the flo_humanoid URDF meshes:
RUN rm /web_app/public/mesh_root/flo_humanoid~master
RUN mkdir -p /web_app/public/mesh_root/flo_humanoid~master/urdf
COPY ./flo_humanoid/urdf /web_app/public/mesh_root/flo_humanoid~master/urdf

RUN npm run build

# Stage 1, based on Nginx, to have only the compiled app, ready for production with Nginx
FROM nginx:1.16
COPY --from=build-web-app /web_app/build/ /usr/share/nginx/html
COPY ./flo_web/nginx.conf /etc/nginx/conf.d/default.conf
