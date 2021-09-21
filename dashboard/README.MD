# PATS-C

Some usefull commands on dash. Replace `docker-compose.yml` with `beta-docker-compose.yml` for the beta server.

## Restart and rebuild the docker:
```
docker-compose -f docker-compose.yml down -v
docker-compose -f docker-compose.yml up --build
```

or for the beta site:

```
docker-compose -f docker-compose.beta_site.yml down -v
docker-compose -f docker-compose.beta_site.yml up --build
```

## Show the terminal output:
`docker logs --follow dashboard_pats_c_1`

## Start a shell inside the pats-c docker
`docker exec -ti dashboard_pats_c_1 sh`

## Initialise a new server:
 Run the init letsencrypt script (possibly with sudo?) for the correct server type (normal/beta)
