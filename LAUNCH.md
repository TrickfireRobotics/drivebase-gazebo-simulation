# Launch steps

## 0. Build the container

```bash
scripts/build.sh
```

## 1. Attach to the container

```bash
scripts/run_macos.sh
```

## 2. DOCKER: Run gazebo

```bash
scripts/run_gazebo.sh
```

## 3. DOCKER: Spawn the model

```bash
scripts/spawn_model.sh
```

## 4. DOCKER: Start the Gazebo x Ros bridge

```bash
scripts/start_bridge.sh
```
