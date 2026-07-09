"""Inspire RH56DFTP-2 hand integration for the G1 MuJoCo sim.

Dieses Paket bringt den (urspruenglich eigenstaendigen) ftp_hand_controller in
das g1pilot-Sim-Setup. Statt per Modbus TCP mit der echten Hand zu reden, spricht
die Bridge jetzt ueber ein austauschbares Backend mit der Simulation.

Stufen-Plan (siehe README im selben Ordner):
  * Stufe 1 (aktiv): ``SimJointStateBackend`` -> Finger-Gelenke nach /joint_states
    (RViz folgt der Steuerung). Taktil-/Kraftwerte = 0.
  * Stufe 2 (vorbereitet): ``MujocoContactBackend`` -> echte MuJoCo-Physik mit
    Kollisionen + aus Kontaktkraeften gefakte Taktil-/Kraftwerte.
"""
