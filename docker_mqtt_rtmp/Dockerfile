FROM tiangolo/nginx-rtmp

# Instala Mosquitto
RUN apt-get update && apt-get install -y mosquitto && rm -rf /var/lib/apt/lists/*

# Copia arquivos de configuração
COPY mosquitto.conf /etc/mosquitto/mosquitto.conf
COPY nginx.conf /etc/nginx/nginx.conf

# Documenta portas expostas
EXPOSE 1883 1935

# Inicia os dois servidores
CMD bash -c "\
  mosquitto -c /etc/mosquitto/mosquitto.conf & \
  nginx -g 'daemon off;'"
