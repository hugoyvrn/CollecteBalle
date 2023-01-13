# Tennis Ball Collector

Ceci est un template de dépôt Git pour le cours d'ingénierie système et modélisation robotique à l'ENSTA Bretagne en 2023.


## Lancer la simulation

### Dépendences


### Démarrer la simulation

Pour lancer la détection des balles et des zones de dépôt à partir de l'image issue de la caméra, éxécutez les commandes suivantes :

```bash
colcon build --packages-select process_camera_pkg
. install/setup.bash
ros2 run process_camera_pkg process_camera_img --ros-args -p display_mode:=False
```

<ins>Note:</ins> Modifiez la valeur de display_mode ('True' ou 'False') pour choisir d'afficher les images issuent du traitement des données de la caméra (voir le terrain ainsi que les balles et zones détectées par le programme) 

## Groupe

### Membres

Damien Esnault
Mirado Rajaomarosata
Nicolas Defour
Maël Godard
Hugo Yverneau 


### Gestion de projet

https://tree.taiga.io/project/d_snlt_work_account-federer-reconversions/admin/project-profile/details

## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.


### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow_fork.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
