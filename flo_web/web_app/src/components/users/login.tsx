import React from "react";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";
import axios from "axios";
import { useHistory, Link } from "react-router-dom";

const loginSchema = Yup.object().shape({
  email: Yup.string()
    .email("invalid email")
    .required("required"),
  password: Yup.string().required("required")
});

interface LoginProps {
  setLoggedIn: (arg: boolean) => void;
  setUserName: (arg: string) => void;
  setUserType: (arg: string) => void;
}

const Login: React.FunctionComponent<LoginProps> = ({
  setLoggedIn,
  setUserName,
  setUserType
}) => {
  const history = useHistory();

  return (
    <div>
      <h1>Login</h1>
      <div>
        <Link to="/register">Register</Link>
      </div>
      <br />
      <div>
        <Formik
          initialValues={{ email: "", password: "" }}
          validationSchema={loginSchema}
          onSubmit={(values, { setSubmitting }): void => {
            axios
              .post("/api/users/login", {
                email: values.email,
                password: values.password
              })
              .then(
                res => {
                  console.log(res.data["userName"]);
                  setLoggedIn(true);
                  setUserType(res.data["userType"]);
                  setUserName(res.data["userName"]);
                  history.push("/");
                },
                err => {
                  alert("failed login");
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

export default Login;
