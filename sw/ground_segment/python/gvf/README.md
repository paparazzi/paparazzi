For details about how to employ the Python scripts check the wiki entry
https://wiki.paparazziuav.org/wiki/Module/guidance_vector_field

## gvfApp

`gvfApp.py` draws an aircraft following the guidance vector vield:

```bash
python gvfApp.py 80
```

Draws for the aircraft with id 80.

## circularFormation

`circularFormation.py` control several aircrafts in a circular formation with constant speeds:

```bash
python circularFormation.py ./formation/two_aircraft.json [fixedwing|rotorcraft]
```

Recommended reading for proper understanding of circular formations:
https://arxiv.org/abs/1703.07736