import users from './users';
import robots from './robots';
import permissions from './permissions';
import webrtc from './webrtc';
import express from 'express';

const exp = (app: express.Application) => {
    app.get('/api', (req, resp) => {
        resp.send('you have reached the api server');
    });
    app.use('/api/users', users);
    app.use('/api/robots', robots);
    app.use('/api/permissions', permissions);
    app.use('/api/webrtc', webrtc);
};

export default exp;
