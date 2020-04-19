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

const saltRounds = 10;

router.post('/add', checkAdmin, async (req, res) => {
    const { robotName, email } = req.body;

    try {
        const { rows } = await db.query(
            'insert into robot_permissions ' +
                '(user_id, robot_id) ' +
                'values ' +
                '((select id from users where email=$1), ' +
                '(select id from robots where robot_name=$2));',
            [email, robotName],
        );
    } catch (e) {
        res.status(400).json({
            error: e,
        });
        return;
    }
    res.status(200).json({
        success: 'gave permissions',
        robotName: robotName,
        email: email,
    });
});
