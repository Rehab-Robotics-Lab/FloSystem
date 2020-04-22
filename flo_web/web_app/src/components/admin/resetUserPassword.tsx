import axios from "axios";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";
import React, { useState } from "react";

const resetPasswordSchema = Yup.object().shape({
  email: Yup.string()
    .email("invalid email")
    .required("required")
});

const ResetUserPassword: React.FunctionComponent = () => {
  const [newPassword, setNewPassword] = useState<string | undefined>(undefined);

  return (
    <div>
      <h1>Reset Users Password</h1>
      <div>
        <Formik
          initialValues={{ email: "" }}
          validationSchema={resetPasswordSchema}
          onSubmit={(values, { setSubmitting }): void => {
            axios
              .post("/api/users/reset-password", {
                email: values.email
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
              <label htmlFor="email">e-mail</label>
              <Field type="email" name="email" />
              <ErrorMessage name="email" component="div" />
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

export default ResetUserPassword;
