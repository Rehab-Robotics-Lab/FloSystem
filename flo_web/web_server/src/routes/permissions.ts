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

router.post('/remove', checkAdmin, async (req, res) => {
    const { robotName, email } = req.body;

    try {
        const { rows } = await db.query(
            'delete from robot_permissions ' +
                'where ' +
                'robot_id =(select id from robots where robot_name=$1) and ' +
                'user_id=(select id from users where email = $2)',
            [robotName, email],
        );
    } catch (e) {
        res.status(400).json({
            error: e,
        });
        return;
    }
    res.status(200).json({
        success: 'removed permissions',
        robotName: robotName,
        email: email,
    });
});
router.get('/', checkAdmin, async (req, res) => {
    const userID = req.session!.userID;
    try {
        const { rows } = await db.query(
            'select r.robot_name ,array_agg(u.email) users from robots r ' +
                'left join robot_permissions rp on rp.robot_id = r.id ' +
                'left join users u on u.id=rp.user_id  ' +
                'group by r.robot_name  ',
            [],
        );
        res.status(200).json({ permissions: rows });
    } catch (e) {
        res.status(500).json({ error: e.toString() });
    }
});
