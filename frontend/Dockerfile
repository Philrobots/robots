FROM node:15.6.0

WORKDIR /usr/src/app

COPY package.json .
COPY yarn.lock .

RUN yarn install

EXPOSE 5000

COPY public public
COPY src src

COPY .browserslistrc .
COPY .eslintrc.js .
COPY .prettierrc.json .
COPY babel.config.js .
COPY jest.config.js .
COPY tsconfig.json .

CMD ["yarn", "serve"]
