# PATS-C

Some usefull commands on dash. Replace `docker-compose.yml` with `beta-docker-compose.yml` for the beta server.

## Start a shell inside the pats-c docker
`docker exec -ti dashboard_pats_c_1 sh`

## Restart the docker:
`docker-compose -f docker-compose.yml restart`

## Restart the docker harder and more manually:
```
docker-compose -f docker-compose.yml down -v
docker-compose -f docker-compose.yml up --build
```

## Show the terminal output:
`docker logs --follow dashboard_pats_c_1`
