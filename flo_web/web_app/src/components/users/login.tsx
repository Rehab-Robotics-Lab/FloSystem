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
  loggedIn: boolean;
}

const Login: React.FunctionComponent<LoginProps> = ({
  setLoggedIn,
  setUserName,
  setUserType,
  loggedIn
}) => {
  const history = useHistory();
  if (loggedIn) {
    history.push("/");
  }

  return (
    <div>
      <h1>Login</h1>
      <div>
        <Link to="/register">Register</Link>
      </div>
      <br />
      <div style={{ display: "inline-block" }}>
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
                  setLoggedIn(true);
                  setUserType(res.data["userType"]);
                  setUserName(res.data["userName"]);
                  history.push("/");
                },
                err => {
                  console.log(err);
                  alert("failed login: " + err.response.data["error"]);
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
                  <label htmlFor="email">e-mail: </label>
                </div>
                <div>
                  <Field type="email" name="email" />
                  <ErrorMessage name="email" component="div" />
                </div>
                <div style={{ textAlign: "left" }}>
                  <label htmlFor="password">password: </label>
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

export default Login;
