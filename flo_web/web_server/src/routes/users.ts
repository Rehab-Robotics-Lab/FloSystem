import Router from 'express-promise-router';
import * as db from '../db';
import bcrypt from 'bcrypt';
import * as EmailValidator from 'email-validator';
import express from 'express';
import crypto from 'crypto';
// create a new express-promise-router
// this has the same API as the normal express router except
// it allows you to use async functions as route handlers
const router = Router();
// export our router to be mounted by the parent application
export default router;

const saltRounds = 10;

const checkLoggedIn: express.RequestHandler = (req, res, next) => {
    if (req.session!.userID === undefined) {
        res.status(400).json({ error: 'you are not logged in' });
        return;
    }
    next();
};

const checkLoggedOut: express.RequestHandler = (req, res, next) => {
    if (req.session!.userID !== undefined) {
        res.status(400).json({ error: 'you are already logged in' });
        return;
    }
    next();
};

const checkAdmin: express.RequestHandler = (req, res, next) => {
    if (req.session!.userID === undefined) {
        res.status(400).json({ error: 'you are not logged in' });
        return;
    }
    if (req.session!.userType !== 'administrator') {
        res.status(400).json({ error: 'you are not authorized' });
        return;
    }
    next();
};

router.post('/login', checkLoggedOut, async (req, res) => {
    const { email, password } = req.body;
    try {
        const {
            rows,
        } = await db.query(
            'select password_hash, u.id, ut.user_type  from users u' +
                ' inner join user_types ut on u.user_type =ut.id' +
                ' where email =$1',
            [email],
        );
        const passwordHash = rows[0]['password_hash'];
        const validPassword = await bcrypt.compare(password, passwordHash);
        if (!validPassword) {
            res.status(401).json({ error: 'invalid password' });
            return;
        }
        req.session!.userID = rows[0]['id'];
        req.session!.userType = rows[0]['user_type'];
        res.status(200).json({ success: 'succesfully logged in' });
        return;
    } catch {
        res.status(401).json({ error: 'error logging in' });
        return;
    }
});

router.post('/logout', checkLoggedIn, async (req, res) => {
    req.session!.destroy((err) => {
        res.clearCookie('connect.sid');

        if (err) {
            res.status(500).json({ error: 'something went wrong' });
            return;
        }
        res.status(200).json({ success: 'logged out' });
    });
});

router.post('/register', checkLoggedOut, async (req, res) => {
    const { firstName, lastName, email, password } = req.body;
    let passwordHash;
    const validEmail = EmailValidator.validate(email);
    if (!validEmail) {
        res.status(400).json({
            error: 'your email (' + email + ') is not valid',
        });
        return;
    }

    if (firstName === undefined) {
        res.status(400).json({ error: 'you must provide a first name' });
        return;
    }

    if (lastName === undefined) {
        res.status(400).json({ error: 'you must provide a last name' });
        return;
    }

    if (password === undefined || password.length < 5) {
        res.status(400).json({
            error: 'you must provide a password of at least 5 characters',
        });
        return;
    }

    try {
        const salt = await bcrypt.genSalt(saltRounds);
        passwordHash = await bcrypt.hash(password, salt);
    } catch (e) {
        res.status(500).json({ error: 'error when hashing new password' });
        return;
    }
    try {
        await db.query(
            'insert into users (first_name, last_name, email, password_hash) values ($1, $2, $3, $4)',
            [firstName, lastName, email, passwordHash],
        );
    } catch (e) {
        if (e.code === '23505') {
            res.status(500).json({ error: 'user already exists' });
            return;
        }
        res.status(500).json({ error: e.detail });
    }

    res.status(200).json({ success: 'added user to system' });
});

router.post('/change-password', checkLoggedIn, async (req, res) => {
    const { oldPassword, newPassword } = req.body;
    try {
        const {
            rows,
        } = await db.query('select password_hash from users where id =$1', [
            req.session!.userID,
        ]);
        const passwordHash = rows[0]['password_hash'];
        const validPassword = await bcrypt.compare(oldPassword, passwordHash);
        if (!validPassword) {
            res.status(401).json({ error: 'invalid password' });
            return;
        }
        const salt = await bcrypt.genSalt(saltRounds);
        const newPasswordHash = await bcrypt.hash(newPassword, salt);
        await db.query('update users set password_hash =$1 where id=$2', [
            newPasswordHash,
            req.session!.userID,
        ]);
        res.status(200).json({ success: 'Password succesfully changed' });
    } catch {
        res.status(500).json({
            error: 'there was an error while changing password',
        });
    }
});

router.post('/reset-password', checkAdmin, async (req, res) => {
    const { email } = req.body;
    try {
        const passwordHex = await crypto.randomBytes(12);
        const password = passwordHex.toString('hex');
        const salt = await bcrypt.genSalt(saltRounds);
        const newPasswordHash = await bcrypt.hash(password, salt);
        await db.query('update users set password_hash =$1 where email=$2', [
            newPasswordHash,
            email,
        ]);
        res.status(200).json({
            success: 'Password succesfully changed',
            newPassword: password,
        });
    } catch {
        res.status(500).json({
            error: 'there was an error while changing password',
        });
    }
});
