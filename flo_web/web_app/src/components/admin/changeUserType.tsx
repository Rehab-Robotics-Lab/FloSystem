import axios from "axios";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";
import React from "react";

const changeUserTypeSchema = Yup.object().shape({
  email: Yup.string()
    .email("invalid email")
    .required("required"),
  userType: Yup.string()
    .matches(
      /(standard|administrator)/,
      "must be of type standard or administrator"
    )
    .required("required")
});

const ChangeUserType: React.FunctionComponent = () => {
  return (
    <div>
      <h1>Change User Type </h1>
      <div>
        <Formik
          initialValues={{ email: "", userType: "standard" }}
          validationSchema={changeUserTypeSchema}
          onSubmit={(values, { setSubmitting }): void => {
            axios
              .post("/api/users/change-type", {
                email: values.email,
                userType: values.userType
              })
              .then(
                res => {
                  alert("sucess!");
                  setSubmitting(false);
                },
                err => {
                  alert(
                    "failed to change user type: " + err.response.data["error"]
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
              <label htmlFor="userType">User Type:</label>
              <Field as="select" name="userType">
                <option value="administrator">administrator</option>
                <option value="standard">standard</option>
              </Field>
              <ErrorMessage name="userType" component="div" />
              <button type="submit" disabled={isSubmitting}>
                Change User Type
              </button>
            </Form>
          )}
        </Formik>
      </div>
    </div>
  );
};

export default ChangeUserType;
