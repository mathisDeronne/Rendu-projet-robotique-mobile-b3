# Projet Robotique Mobile
> Mathis Deronne, Guillaume Landfroid-nazac, Melkiade Ngnintedem

Voici notre rendu du projet de robotique mobile. Il est possible de le lancer en simulation 3D et sur un vrai hexapode d'Ynov.

Pas de panique ! On va vous dire comment lancer la simulation ou faire marcher le robot physiquement.

## Lancer le projet en simulation 3D
### Installation
Pour installer le projet sur votre pc, vous aurez besoin d'installer [git](https://git-scm.com/downloads) et d'ouvrir un terminal a l'endroit ou vous voulez installer les fichiers du projet, puis taper la commande 
```
git clone https://github.com/mathisDeronne/Rendu-projet-robotique-mobile-b3.git
```


Tout d'abord il faudra installer [python](https://www.python.org/downloads/) et les librairies pour lancer la simulation.
> Nous utilisons la version 3.13.3 de Python

Pour cela, on utilise la commande ```pip Install [Nom de la librairie]```.

les librairies a installer sont :

```
numpy
pygame
pybullet
onshape-to-robot
transforms3d
scipy
```
**il se peut que les librairies dépendent d'autres librairies, il faudra les installer aussi avec la commande ```pip install```.**



### Lancement
Maintenant que tout est installé, vous pouvez lancer le projet en tapant la commande ```python main.py``` dans votre terminal et sélectionner le mode que vous voulez sur l'interface graphique.

Dans la simulation de l'haxapode lancée, vous pouvez controler ses déplacements grâce aux touches du clavier Z (pour avancer), Q (aller a gauche), S (reculer), D (aller a droite).


## Lancer le projet sur un hexapode réel

Pour lancer le programme sur hexapode réel il faut relancer le programme ```main``` comme précédemment et il faut cliquer sur **"mode réel"** et sélectionner le mode que vous voulez.

Lorsque vous avez fini avec un mode, cliquez sur **"arreter le mode actuel"** pour l'arreter