FROM node:14
#LTS at time of creation

#create folder fo app
WORKDIR /usr/src/app

#Pull in the package defs
COPY package*.json ./

#Install the packages
RUN npm install

EXPOSE 3000

CMD ["npm", "run","robot"]
