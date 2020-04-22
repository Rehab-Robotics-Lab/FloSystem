import axios from "axios";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";
import React, { useState } from "react";

const resetPasswordSchema = Yup.object().shape({
  name: Yup.string().required("required")
});

const ResetRobotPassword: React.FunctionComponent = () => {
  const [newPassword, setNewPassword] = useState<string | undefined>(undefined);

  return (
    <div>
      <h1>Reset Robot&apos;s Password</h1>
      <div>
        <Formik
          initialValues={{ name: "" }}
          validationSchema={resetPasswordSchema}
          onSubmit={(values, { setSubmitting }): void => {
            axios
              .post("/api/robots/new-password", {
                robotName: values.name
              })
              .then(
                res => {
                  const password = res.data["newPassword"];

                  setNewPassword(password);
                  setSubmitting(false);
                },
                err => {
                  alert(
                    "failed to reset users password: " +
                      err.response.data["error"]
                  );
                  setSubmitting(false);
                }
              );
          }}
        >
          {({ isSubmitting }) => (
            <Form>
              <label htmlFor="name">Robot Name</label>
              <Field type="name" name="name" />
              <ErrorMessage name="name" component="div" />
              <button type="submit" disabled={isSubmitting}>
                Reset Password
              </button>
            </Form>
          )}
        </Formik>
        {newPassword && (
          <div>
            <p>New password (copy this somewhere):</p>
            <ul>
              <li>Password: {newPassword}</li>
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default ResetRobotPassword;
