import React, { useState, useEffect } from "react";
import axios from "axios";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";

const addUserSchema = Yup.object().shape({
  email: Yup.string()
    .email("invalid email")
    .required("required")
});

interface Permissions {
  robot_name: string;
  users: string[];
}

const ChangePermissions: React.FunctionComponent = () => {
  const [permissionsArray, setPermissionsArray] = useState<Permissions[]>([]);

  const update = (): void => {
    axios.get("/api/permissions").then(
      res => {
        setPermissionsArray(res.data.permissions as Array<Permissions>);
      },
      err => {
        console.log("error: " + err.response.data["error"]);
      }
    );
  };

  useEffect(update, []);

  const permissionsRows = permissionsArray.map(permission => (
    <li key={permission.robot_name}>
      {permission.robot_name}
      <ul>
        <li>
          <Formik
            initialValues={{ email: "" }}
            validationSchema={addUserSchema}
            onSubmit={(values, { setSubmitting }): void => {
              axios
                .post("/api/permissions/add", {
                  robotName: permission.robot_name,
                  email: values.email
                })
                .then(
                  res => {
                    update();
                    setSubmitting(false);
                  },
                  err => {
                    alert(err);
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
                  Add
                </button>
              </Form>
            )}
          </Formik>
        </li>
        {permission.users.map(user => {
          if (user) {
            return (
              <li key={permission.robot_name + user}>
                <button
                  type="button"
                  onClick={(): void => {
                    console.log("remove " + user);
                    axios
                      .post("/api/permissions/remove", {
                        robotName: permission.robot_name,
                        email: user
                      })
                      .then(() => {
                        update();
                      });
                  }}
                >
                  Remove {user}
                </button>
              </li>
            );
          } else {
            return null;
          }
        })}
      </ul>
    </li>
  ));

  return (
    <div>
      <h1>Change Permissions</h1>
      <div>
        <div
          style={{
            textAlign: "left",
            display: "inline-block",
            margin: "0 auto"
          }}
        >
          <ul>{permissionsRows}</ul>
        </div>
      </div>
    </div>
  );
};

export default ChangePermissions;
