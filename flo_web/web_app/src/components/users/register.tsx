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
      <div style={{ display: "inline-block" }}>
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
              .then(res => {
                alert("user created, please login");
                history.push("/login");
              })
              .catch(err => {
                alert("failed to register: " + err.response.data["error"]);
                setSubmitting(false);
              });
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
                  <label htmlFor="firstName">First Name</label>
                </div>
                <div>
                  <Field type="firstName" name="firstName" />
                  <ErrorMessage name="firstName" component="div" />
                </div>
                <div style={{ textAlign: "left" }}>
                  <label htmlFor="lastName">Last Name</label>
                </div>
                <div>
                  <Field type="lastName" name="lastName" />
                  <ErrorMessage name="lastName" component="div" />
                </div>
                <div style={{ textAlign: "left" }}>
                  <label htmlFor="email">e-mail</label>
                </div>
                <div>
                  <Field type="email" name="email" />
                  <ErrorMessage name="email" component="div" />
                </div>
                <div style={{ textAlign: "left" }}>
                  <label htmlFor="password">password</label>
                </div>
                <div>
                  <Field type="password" name="password" />
                  <ErrorMessage name="password" component="div" />
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

export default Register;
