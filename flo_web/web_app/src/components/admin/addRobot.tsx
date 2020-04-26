import React, { useState } from "react";
import { Formik, Form, Field, ErrorMessage } from "formik";
import * as Yup from "yup";
import axios from "axios";

interface Robot {
  name: string;
  password: string;
}

const addRobotSchema = Yup.object().shape({
  robotName: Yup.string()
    .min(4, "too short")
    .required("required"),
  robotType: Yup.string()
    .matches(/(lilflo|simple)/, "must be of type lilflo or simple")
    .required("required")
});

const AddRobot: React.FunctionComponent = () => {
  const [newRobot, setNewRobot] = useState<Robot | undefined>(undefined);
  return (
    <div>
      <h1>Create Robot</h1>
      <div>
        <Formik
          initialValues={{ robotName: "", robotType: "simple" }}
          validationSchema={addRobotSchema}
          onSubmit={(values, { setSubmitting }): void => {
            axios
              .post("/api/robots/new-robot", {
                robotName: values.robotName,
                robotType: values.robotType
              })
              .then(res => {
                const name = res.data["newName"];
                const password = res.data["newPassword"];

                setNewRobot({ name: name, password: password });
                setSubmitting(false);
              })
              .catch(err => {
                alert("Failed to add robot: " + err.response.data["error"]);
                setSubmitting(false);
              });
          }}
        >
          {({ isSubmitting }) => (
            <Form>
              <label htmlFor="robotName">Robot Name</label>
              <Field type="robotName" name="robotName" />
              <ErrorMessage name="robotName" component="div" />
              <label htmlFor="robotType">Robot Type:</label>
              <Field as="select" name="robotType">
                <option value="simple">simple</option>
                <option value="lilflo">lilflo</option>
              </Field>
              <ErrorMessage name="robotType" component="div" />
              <button type="submit" disabled={isSubmitting}>
                Submit
              </button>
            </Form>
          )}
        </Formik>
        {newRobot && (
          <div>
            <p>New robot details (copy these somewhere):</p>
            <ul>
              <li>Name: {newRobot.name}</li>
              <li>Password: {newRobot.password}</li>
            </ul>
          </div>
        )}
      </div>
    </div>
  );
};

export default AddRobot;
