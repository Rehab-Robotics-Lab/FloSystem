import express from 'express';
import bodyParser from 'body-parser';
import mountRoutes from './routes';
import { sessionParser, logger } from './util';

const apiPort = 3030;
const app = express();

app.set('trust proxy', 1); // allows us to use nginx with secure cookie in sessions
// TODO: test if we need this.

app.use(sessionParser);

// Parse the string in the requests into json:
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));
// Load in our routes:
mountRoutes(app);
// start server:
app.listen(apiPort, () => {
    logger.info('API server running on port: ' + apiPort);
});
