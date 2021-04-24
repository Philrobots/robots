# COViRondelle2021

[![Robot CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Robot%20CI/badge.svg)](https://github.com/GLO3013-E4/COViRondelle2021/actions?query=workflow%3A%22Robot+CI%22)
[![Station CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Station%20CI/badge.svg)](https://github.com/GLO3013-E4/COViRondelle2021/actions?query=workflow%3A%22Station+CI%22)
[![Frontend CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Frontend%20CI/badge.svg)](https://github.com/GLO3013-E4/COViRondelle2021/actions?query=workflow%3A%22Frontend+CI%22)
[![Scripts CI](https://github.com/GLO3013-E4/COViRondelle2021/workflows/Scripts%20CI/badge.svg)](https://github.com/GLO3013-E4/COViRondelle2021/actions?query=workflow%3A%22Scripts+CI%22)
[![Code coverage](https://codecov.io/gh/GLO3013-E4/COViRondelle2021/branch/develop/graph/badge.svg?token=9552B5FKYR)](https://codecov.io/gh/GLO3013-E4/COViRondelle2021)
[![Dependabot](https://badgen.net/badge/Dependabot/enabled/green?icon=dependabot)](https://dependabot.com/)

Project of team 4 for course GLO-3013 at Laval University

Our team is called "Robot culinaire"!

We have three main applications and some development scripts, each with their own README : 

- [`robot`](robot) : Python application communicating with `station` using ROS
- [`station`](station) : Python application communicating with `robot` using ROS and `frontend` using websockets
- [`frontend`](frontend) : Vue.js application communicating with `station` using websockets
- [`scripts`](scripts) : Development scripts for testing different functionalities

## Installation

With Docker Compose : 
```shell
docker-compose build
docker-compose build --no-cache # If you have issues with packages not updating or installing
```

Without Docker Compose : refer to each app's README.md file.

## Usage

With Docker Compose :
```shell
docker-compose up
```

If you have issues with the Docker network, use `docker system prune -a`.

Without Docker Compose : refer to each app's README.md file.

Each app will run on : 

- Robot : [localhost:3000](http://localhost:3000) *(CAN CHANGE)*
- Station : [localhost:4000](http://localhost:4000) *(CAN CHANGE)*
- Frontend : [localhost:5000](http://localhost:5000)

## Contributing

Before contributing to the project, please read our [contribution guide](CONTRIBUTING.md). Also, please refer to each app's README.md file.


## License

`MIT` : [Read full license](LICENSE)
