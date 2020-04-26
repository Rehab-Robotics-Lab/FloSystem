import React from "react";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";
import axios from "axios";

const changePasswordSchema = Yup.object().shape({
  oldPassword: Yup.string().required("required"),
  newPassword: Yup.string().required("required")
});

const ChangePassword: React.FunctionComponent = () => {
  return (
    <div>
      <h1>Change Password</h1>
      <div style={{ display: "inline-block" }}>
        <Formik
          initialValues={{ oldPassword: "", newPassword: "" }}
          validationSchema={changePasswordSchema}
          onSubmit={(values, { setSubmitting }): void => {
            axios
              .post("/api/users/change-password", {
                newPassword: values.newPassword,
                oldPassword: values.oldPassword
              })
              .then(
                res => {
                  alert("succesfully changed password");
                  setSubmitting(false);
                },
                err => {
                  alert(
                    "failed to change password: " + err.response.data["error"]
                  );
                  setSubmitting(false);
                }
              );
          }}
        >
          {({ isSubmitting }) => (
            <Form>
              <div
                style={{
                  display: "flex",
                  justifyContent: "center",
                  flexDirection: "column"
                }}
              >
                <div style={{ textAlign: "left" }}>
                  <label htmlFor="oldPassword">Old Password: </label>
                </div>
                <div>
                  <Field type="oldPassword" name="oldPassword" />
                  <ErrorMessage name="oldPassword" component="div" />
                </div>
                <div style={{ textAlign: "left" }}>
                  <label htmlFor="newPassword">New Password: </label>
                </div>
                <div>
                  <Field type="newPassword" name="newPassword" />
                  <ErrorMessage name="newPassword" component="div" />
                </div>
                <div>
                  <button type="submit" disabled={isSubmitting}>
                    Submit
                  </button>
                </div>
              </div>
            </Form>
          )}
        </Formik>
      </div>
    </div>
  );
};

export default ChangePassword;
