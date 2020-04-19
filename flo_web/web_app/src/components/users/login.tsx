import React from "react";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";

const loginSchema = Yup.object().shape({
  email: Yup.string()
    .email("invalid email")
    .required("required"),
  password: Yup.string().required("required")
});

const Login: React.FunctionComponent = () => {
  return (
    <div>
      <h1>Login</h1>

      <Formik
        initialValues={{ email: "", password: "" }}
        validationSchema={loginSchema}
        onSubmit={(values, { setSubmitting }): void => {
          console.log(values);
        }}
      >
        {({ isSubmitting }) => (
          <Form>
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
  );
};

export default Login;
