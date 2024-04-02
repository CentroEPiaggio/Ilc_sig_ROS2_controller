<<<<<<< HEAD
# Framework Planner+ILC

Il framework Planner+ILC proposto opera secondo i seguenti passaggi:

1. Il planner in CasADi genera traiettorie cartesiane e le salva in file `.mat`.
   
2. Utilizzando MATLAB, è possibile caricare il file `.mat`, visualizzare le traiettorie cartesiane e, se soddisfacenti, eseguire l'inverse kinematics (IK) per generare due file CSV:
    - `init.csv`: che guida il robot dalla posizione di partenza (tutti zeri) alla configurazione iniziale `q0` del task (la quale può variare in base alla configurazione iniziale del robot durante la pianificazione).
    - `task.csv`: contenente il task effettivo.

   Entrambe le traiettorie vengono ricampionate in base alla frequenza di campionamento del controllore.

3. I file `task.csv` e `init.csv` devono essere copiati nella cartella `/csv_task` del pacchetto `ilc_sig_controller` (o un percorso analogo), sostituendo eventuali file presenti con gli stessi nomi. Il controllore caricherà i dati direttamente da questo percorso.

**IMPORTANTE**: È necessario configurare sempre il parametro `period` nel file YAML del control manager. Questo parametro deve essere uguale al periodo di campionamento sia della traiettoria che del controllore.
=======
# Ilc_sig_ROS2_controller
>>>>>>> 9f39bb541d08d98b65ab1fc5f5067e6663b59d75
