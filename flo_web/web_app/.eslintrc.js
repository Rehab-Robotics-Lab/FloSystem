module.exports = {
    "env": {
        "browser": true,
        "es6": true
    },
    "extends": [
        "airbnb",
        "plugin:@typescript-eslint/recommended",
        "plugin:react/recommended",
        //"prettier/@typescript-eslint",
        //"plugin:prettier/recommended"
    ],
    "globals": {
        "Atomics": "readonly",
        "SharedArrayBuffer": "readonly"
    },
    "parser": "@typescript-eslint/parser",
    "parserOptions": {
        "ecmaFeatures": {
            "jsx": true
        },
        "ecmaVersion": 2018,
        "sourceType": "module",
        "project":"./tsconfig.json"
    },
    "plugins": [
       "react",
        "@typescript-eslint",
        "prettier"
    ],
    "rules": {
        //"prettier/prettier":"error"
    }
}
