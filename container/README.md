# Container execution

Build the image from the repository root:

```bash
docker build -f container/Dockerfile -t adaptive-nonmanifold-meshing .
```

Run the maintained Eistute regression:

```bash
docker run --rm adaptive-nonmanifold-meshing
```

The container is a convenience environment for the same public command-line application and validation scripts available in the repository.
