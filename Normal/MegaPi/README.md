# Code MegaPi

Ce code ouvre une communication série et écoute les messages envoyés par port série par le RPI.

# Modifications librairie MeMegaPi.h
Des modifications ont été apportées à la cette librairie car elle pouvait causer un bug (inclure deux fois la lib dans deux fichiers séparés donnaient une erreur de compilation).
La modification consiste en la déclaration des variables mePort, encoder_Port, megapi_dc_Port et megaPi_slots dans un fichier MeMegaPiGlobals.cpp et en déclaration externe dans le fichier MeMegaPi.h. Auparavant, la déclaration se faisait directement dans le header, ce qui est une mauvaise pratique.