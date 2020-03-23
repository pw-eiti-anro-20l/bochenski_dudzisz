# ANRO_laboratorium_1


## Zadanie
Zadanie laboratoryjne polegało na stworzeniu programu sterującego żółwiem z pakietu turtlesim oprogramowania ROS. Program ma zapewnić sterowanie przy pomocy wybranych klawiszy wczytywanych z serwera parametrów startującego automatycznie po uruchomieniu roscore.

## Instrukcja użytkownika
W komputerze z systemem Linux i zainstalowanymi pakietami ROS ([oficjalny poradnik konfiguracji](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)) należy:
1. Pobrać pakiet anro z repozytorium git do folderu roboczego programu catkin.
2. W jednym terminalu uruchomić roscore poleceniem `roscore`.
3. W osobnym terminalu, w lokalizacji folderu roboczego programu catkin, wpisać  `source ./devel/setup.bash`, a następnie `roslaunch anro turtle.launch`.
4. Pojawi się okno programu TurtleSim. Jako przyciski sterujące żółwiem zostały wybrane WSAD odpowednio poruszające żółwiem do przodu, tyłu, w lewo i w prawo. Należy pamiętać, że aby działały, aktywnym oknem powinno być okno terminala, w którym został uruchomiony plik turtle.lauch.
5. Program należy wyłączyć kombinacją klawiszy w `ctrl + C` będąc w terminalu, z którego został uruchomiony.

## Dokumentacja
Jako język programowania wybrany został Python. 

### Plik turtle.launch
W pliku turtle.launch znajduje się się opis stworzonego systemu. Składa się z część definiującej węzeł żółwia z pakietu turtlesim, którym będziemy sterować (subscriber):

```<node pkg="turtlesim" name="turtle" type="turtlesim_node"/>```

oraz węzeł control z utworzonego pakietu anro, który będzie wczytywał wpisane przez użytkownika znaki i wysyłał odpowiednie sygnały sterujące do węzła turtle:

```node pkg="anro" name="control" type="turtle_control.py" output="screen">```

(zawierający publisher). Jednocześnie definiowany jest słownik sygnałów sterujących dla serwera parametrów:

```
<param name="forward" value="w"/> 
<param name="backward" value="s"/>
<param name="left" value="a"/>
<param name="right" value="d"/>
```

### Plik turtle_control.py - publisher
Plik zawiera główny kod programu w języku Python. W pierwszych liniach importowane są niezbędne pakiety. Następnie definiowane są funkcje.

* getKey() - odczytuje i zwraca odczytany z klawiatury pojedynczy wciśnięty klawisz.
* turtle_control() - główna funkcja programu, zawierającą jego główną pętlę. Na początku jednorazowo przy uruchomieniu programu definiowany jest węzeł publikujący wiadomości oraz odczytywane są parametry z serwera parametrów:

```
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
rospy.init_node('turtle_control', anonymous=True)
control_keys = rospy.get_param("/control/")
```

Wypisywane są odpowiednie komunikaty dla użytkownika i rozpoczyna się pętla, w której kolejno:
* Tworzona jest wiadomość typu Twist.
* Odczytywany jest klawisz z klawiatury.
* W zależności od wczytanego klawisza, zgodnie z ich mapowaniem w serwerze parametrów, modyfikowane są odpowiednie pola wiadomości Twist (`twist.linear.x` lub `twist.angular.z`).
* Wysyłana jest wiadomość Twist.
* Sprawdzany jest warunek kombinacji klawiszy kończącej działanie programu.

Funkcja `turtle_control()`dla bezpieczeństwa wykonywana jest w bloku `try`.

### Struktura systemu
System składa się z dwóch węzłów pełniących rolę subskrybenta (subscriber) - żółw turtle, którym sterujemy, i nadawcy sygnałów (publisher) control - program odczytujący z klawiatury znaki i wysyłające je do subskrybentów (w tym przypadku jednego subskrybenta). Schemat komunikacji wygenerowany programem rqt_graph wygląda następująco:

![Communication scheme](https://raw.githubusercontent.com/pw-eiti-anro-20l/bochenski_dudzisz/master/communication.png)

Węzeł `/control` komunikuje się z węzłem `/turtle` przez wiadomości wysyłane na temat `/turtle1/cmd_vel`.
