import React from "react";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";
import axios from "axios";
import { useHistory, Link } from "react-router-dom";

const registerSchema = Yup.object().shape({
  email: Yup.string()
    .email("invalid email")
    .required("required"),
  password: Yup.string()
    .min(5, "too short")
    .required("required"),
  firstName: Yup.string().required("required"),
  lastName: Yup.string().required("required")
});
const Register: React.FunctionComponent = () => {
  const history = useHistory();

  return (
    <div>
      <h1>Register</h1>
      <div>
        <Link to="/login">Login</Link>
      </div>
      <br />
      <div>
        <Formik
          initialValues={{
            email: "",
            password: "",
            firstName: "",
            lastName: ""
          }}
          validationSchema={registerSchema}
          onSubmit={(values, { setSubmitting }): void => {
            axios
              .post("/api/users/register", {
                email: values.email,
                password: values.password,
                firstName: values.firstName,
                lastName: values.lastName
              })
              .then(
                res => {
                  alert("user created, please login");
                  history.push("/login");
                },
                err => {
                  alert("failed to register");
                  setSubmitting(false);
                }
              );
          }}
        >
          {({ isSubmitting }) => (
            <Form>
              <label htmlFor="firstName">First Name</label>
              <Field type="firstName" name="firstName" />
              <ErrorMessage name="firstName" component="div" />
              <label htmlFor="lastName">Last Name</label>
              <Field type="lastName" name="lastName" />
              <ErrorMessage name="lastName" component="div" />
              <label htmlFor="email">e-mail</label>
              <Field type="email" name="email" />
              <ErrorMessage name="email" component="div" />
              <label htmlFor="password">password</label>
              <Field type="password" name="password" />
              <ErrorMessage name="password" component="div" />
              <button type="submit" disabled={isSubmitting}>
                Submit
              </button>
            </Form>
          )}
        </Formik>
      </div>
    </div>
  );
};

export default Register;
