import Router from 'express-promise-router';
import * as db from '../db';
import bcrypt from 'bcrypt';
import crypto from 'crypto';
import { checkLoggedIn, checkAdmin } from './users';
// create a new express-promise-router
// this has the same API as the normal express router except
// it allows you to use async functions as route handlers
const router = Router();
// export our router to be mounted by the parent application
export default router;

const coturn_secret = process.env.COTURN_SECRET || 'coturn_secret';

// From: https://stackoverflow.com/questions/35766382/coturn-how-to-use-turn-rest-api
function getTURNCredentials(name: string, secret: string) {
    const unixTimeStamp = Math.ceil(Date.now() / 1000) + 60 * 60; // this credential would be valid for the next 1 hour
    const username = [unixTimeStamp, name].join(':');
    const hmac = crypto.createHmac('sha1', secret);
    hmac.setEncoding('base64');
    hmac.write(username);
    hmac.end();
    const password = hmac.read();
    return {
        username: username,
        password: password,
    };
}

router.get('/turn-credentials', checkLoggedIn, async (req, res) => {
    const { robotName, username, password } = req.body;
    let ctname: string;
    try {
        if (robotName !== undefined) {
            const id = req.session!.userID;
            const {
                rows,
            } = await db.query(
                'select count(*) from robot_permissions rp ' +
                    'left join robots r on r.id = rp.robot_id ' +
                    'where rp.user_id =$1 and r.robot_name =$2',
                [id, robotName],
            );

            if (rows[0] < 1) {
                res.status(403).json({ error: 'insufficient permissions' });
                return;
            }
            ctname = id;
        } else if (username !== undefined && password !== undefined) {
            const {
                rows,
            } = await db.query(
                'select password_hash from robots where robot_name=$1',
                [username],
            );
            const validPassword = await bcrypt.compare(
                password,
                rows[0]['password_hash'],
            );

            if (!validPassword) {
                res.status(401).json({ error: 'failed to auth' });
                return;
            }

            ctname = username;
        } else {
            res.status(400).json({ error: 'nonsensical request' });
            return;
        }

        const coturn_creds = getTURNCredentials(ctname, coturn_secret);

        res.status(200).json(coturn_creds);
    } catch (e) {
        res.status(500).json({ error: e });
    }
});
