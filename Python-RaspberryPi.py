# ===== Bibliotheken Importeren =====
# Importeert de Paho MQTT-bibliotheek, een populaire client voor MQTT-communicatie in Python.
# We geven het de kortere naam 'mqtt' voor gebruiksgemak.
import paho.mqtt.client as mqtt
# Importeert de client-bibliotheek voor InfluxDB, een database die geoptimaliseerd is voor tijdgebonden data (zoals sensormetingen).
from influxdb import InfluxDBClient
# Importeert de 'time' bibliotheek (hier niet direct gebruikt, maar vaak handig in dit soort scripts).
import time
# Importeert het 'datetime' object uit de 'datetime' bibliotheek om met timestamps te kunnen werken.
from datetime import datetime

# ===== Configuratie =====
# Hier worden de instellingen gedefinieerd die het script gebruikt om verbinding te maken.
MQTT_BROKER = "192.168.1.42"  # Het IP-adres van de MQTT-broker (de server die berichten ontvangt en doorstuurt).
MQTT_PORT = 1883              # De standaard netwerkpoort voor MQTT-communicatie.
INFLUX_HOST = "localhost"     # Het adres van de InfluxDB-server. 'localhost' betekent dat deze op dezelfde machine draait als dit script.
INFLUX_DB = "test_mqtt"       # De naam van de database binnen InfluxDB waar de data wordt opgeslagen.

# ===== InfluxDB Setup =====
# Creëert een InfluxDB-client object. Dit object wordt gebruikt om verbinding te maken met de database en er data naartoe te schrijven.
influx = InfluxDBClient(host=INFLUX_HOST, database=INFLUX_DB)

# ===== MQTT Callbacks =====
# Een callback-functie wordt automatisch aangeroepen wanneer een specifieke gebeurtenis plaatsvindt.

# Deze functie wordt aangeroepen zodra de client succesvol verbinding heeft gemaakt met de MQTT-broker.
def on_connect(client, userdata, flags, rc, properties):
    # Print een bevestiging en de resultaatcode (rc=0 betekent een succesvolle verbinding).
    print(f"Verbonden met MQTT (code: {rc})")
    # Abonneert de client op een lijst van topics. Vanaf nu zal de 'on_message' functie worden aangeroepen voor berichten op deze topics.
    # De '0' staat voor Quality of Service level 0, wat betekent "verstuur maximaal één keer".
    client.subscribe([
        ("robot/status", 0),         # Statusberichten van de robot (bv. "Lijnvolgen").
        ("robot/sensors/voor", 0),   # Data van de voorste sensor.
        ("robot/sensors/rechts", 0), # Data van de rechter sensor.
        ("robot/drukknop", 0)        # Status van de fysieke drukknop op de robot.
    ])

# Deze functie wordt aangeroepen telkens wanneer een bericht binnenkomt op een van de geabonneerde topics.
def on_message(client, userdata, msg):
    # Een 'try...except' blok vangt eventuele fouten op tijdens het verwerken van een bericht, zodat het script niet crasht.
    try:
        # Maakt een timestamp in UTC-formaat. InfluxDB werkt het beste met UTC.
        timestamp = datetime.utcnow().isoformat()
        # Haalt het topic uit het binnenkomende bericht (bv. "robot/status").
        topic = msg.topic
        # Haalt de inhoud (payload) uit het bericht en decodeert deze van bytes naar een leesbare string.
        payload = msg.payload.decode()
        
        # Print het bericht op de console voor directe feedback, met de huidige tijd.
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {topic} : {payload}")
        
        # Controleert van welk topic het bericht afkomstig is en voert de bijbehorende actie uit.
        if topic == "robot/status":
            # Dit is een slimme manier om de status op te slaan. We schrijven naar drie verschillende 'measurements' (tabellen).
            statuses = ["Stoppen", "Lijnvolgen", "Obstakel"] # Definieer de mogelijke statussen.
            
            # Loop door elke mogelijke status.
            for status in statuses:
                # Schrijf een datapunt naar InfluxDB.
                influx.write_points([{
                    "measurement": status.lower(), # De naam van de 'tabel' (bv. 'stoppen').
                    "tags": {"robot": "esp32"},    # Extra metadata om data te filteren.
                    "fields": {
                        # Het veld 'value' krijgt een 1 als de payload overeenkomt met de status, anders een 0.
                        # Hiermee kun je in Grafana makkelijk zien welke status actief was.
                        "value": 1 if payload == status else 0
                    },
                    "time": timestamp # De timestamp van de meting.
                }])
        
        elif topic == "robot/drukknop": # Als het bericht van de drukknop komt...
            # Converteer de payload ('1' of '0') naar een leesbare status voor de console.
            status = "ingedrukt" if payload == "1" else "losgelaten"
            # Schrijf de knopstatus naar een specifieke measurement in InfluxDB.
            influx.write_points([{
                "measurement": "drukknoprobotje",
                "tags": {"component": "button"},
                "fields": {"value": int(payload)}, # Sla de waarde op als een getal (0 of 1).
                "time": timestamp
            }])
            print(f"Knopstatus: {status}") # Print de leesbare status.
            
        elif "sensors" in topic: # Als het bericht van een sensor komt...
            # Controleer welke sensor het is.
            if "voor" in topic:
                # Schrijf de waarde van de voorste sensor naar InfluxDB.
                influx.write_points([{
                    "measurement": "voor_sensor",
                    "tags": {"type": "ultrasonic"},
                    # Converteer de payload naar een 'float' (getal met decimalen).
                    "fields": {"value": float(payload)},
                    "time": timestamp
                }])
            elif "rechts" in topic:
                # Schrijf de waarde van de rechter sensor naar InfluxDB.
                influx.write_points([{
                    "measurement": "rechts_sensor",
                    "tags": {"type": "ultrasonic"},
                    "fields": {"value": float(payload)},
                    "time": timestamp
                }])
            
    except Exception as e:
        # Als er een fout optreedt in het 'try'-blok, print dan een duidelijke foutmelding.
        print(f"Verwerkingsfout: {str(e)}")

# ===== Hoofdprogramma =====
# De 'main' functie bevat de logica om het script op te starten.
def main():
    print("=== Robot Monitoring Systeem ===")
    print("Configuratie:")
    print(f"- MQTT Broker: {MQTT_BROKER}")
    print(f"- InfluxDB Database: {INFLUX_DB}\n")
    
    # Maakt een MQTT-client object aan. `CallbackAPIVersion.VERSION2` is de moderne, aanbevolen versie.
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    # Koppelt onze 'on_connect' functie aan de 'on_connect' gebeurtenis van de client.
    mqtt_client.on_connect = on_connect
    # Koppelt onze 'on_message' functie aan de 'on_message' gebeurtenis van de client.
    mqtt_client.on_message = on_message
    
    try:
        # Probeert verbinding te maken met de MQTT-broker op het opgegeven adres en poort.
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
        print("Start monitoring... (Ctrl+C om te stoppen)\n")
        # Start een oneindige lus die op de achtergrond luistert naar MQTT-berichten.
        # Deze functie blokkeert de uitvoering van de code hierna, totdat het script stopt.
        mqtt_client.loop_forever()
        
    except KeyboardInterrupt:
        # Deze code wordt uitgevoerd als de gebruiker op Ctrl+C drukt om het script te stoppen.
        print("\nAfsluiten...")
    finally:
        # Dit blok wordt altijd uitgevoerd, of het script nu crasht of netjes wordt afgesloten.
        # Het zorgt ervoor dat verbindingen netjes worden gesloten.
        mqtt_client.disconnect()
        influx.close()

# Dit is het startpunt van het script wanneer het wordt uitgevoerd vanuit de command-line.
if __name__ == "__main__":
    # Voordat we starten, zorgen we ervoor dat de database in InfluxDB bestaat.
    # Als deze al bestaat, doet dit commando niets.
    influx.create_database(INFLUX_DB)
    # Roep de hoofdfunctie aan om het monitoringproces te starten.
    main()