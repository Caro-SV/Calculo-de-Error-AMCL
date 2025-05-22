<h1 align="center">Cálculo del Error de Posición y Orientación con AMCL en Gazebo</h1>

<p align="center">
  <i>Este nodo ROS2 compara la posición y orientación estimadas por AMCL con la posición real del modelo en el entorno simulado de Gazebo.</i>
</p>

<h2>Descripción</h2>
<p>El nodo <code>NavComparisonNode</code> permite evaluar la precisión del sistema de localización basado en AMCL (Adaptive Monte Carlo Localization) utilizando los datos publicados por Gazebo como referencia.</p>

<p>Calcula dos métricas principales:</p>
<ul>
  <li><strong>Error de posición</strong>: Distancia euclidiana entre la posición estimada por AMCL y la posición real del robot en Gazebo.</li>
  <li><strong>Error de orientación</strong>: Diferencia angular entre las orientaciones (yaw) de ambas estimaciones.</li>
</ul>

<h2>Funcionamiento General</h2>

<p>El nodo se suscribe a dos tópicos:</p>
<ul>
  <li><code>/amcl_pose</code>: Para recibir la estimación de la localización actual del robot.</li>
  <li><code>/model_states</code>: Para acceder a la ubicación real del modelo en el entorno simulado de Gazebo.</li>
</ul>

<p>Cada segundo, se calcula y publica en consola el error de posición y orientación.</p>

<h2>Ejecución</h2>

<p>Asegúrate de tener corriendo un entorno con obstáculos en Gazebo y el paquete <code>amcl</code> activado.</p>

<p><strong>1. Carga el entorno en Gazebo:</strong></p>
<pre style="background-color:#e8f5e9;padding:10px;border-radius:5px"><code>ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py</code></pre>


<p><strong>2. Lanza la navegación con AMCL y el mapa del entorno:</strong></p>
<pre style="background-color:#e8f5e9;padding:10px;border-radius:5px"><code>ros2 launch nav2_bringup localization_launch.py \
    map:=/ruta/a/tu_mapa.yaml \
    use_sim_time:=true</code></pre>
    
<p align="center">
  <img src="https://github.com/carolinasernav/Calculo-de-Error-AMCL/blob/c986413cfcef28bc9f3f318841f1b905c9a27cbe/Robot%20AMCL.png" alt="Visual de funcionamiento"/>
</p>

<p><strong>3. Ejecuta el nodo de comparación:</strong></p>
<pre style="background-color:#e8f5e9;padding:10px;border-radius:5px"><code>ros2 run <nombre_paquete> AMCL</code></pre>

<p><strong>4. Por último, mueve el robot con <code>/Nav2 Goal</code> a una ubicación objetivo</strong></p>
<p>Los resultados se publicaran en la consola de la siguiente manera:</p>

<p align="center">
  <img src="https://github.com/carolinasernav/Calculo-de-Error-AMCL/blob/c986413cfcef28bc9f3f318841f1b905c9a27cbe/Resultados%20en%20Consola.png" alt="Visual de funcionamiento"/>
</p>

<h2>📂 Estructura del Nodo</h2>
<ul>
  <li><code>amcl_callback</code>: Guarda la pose estimada por AMCL.</li>
  <li><code>gz_callback</code>: Obtiene la pose real del robot desde Gazebo.</li>
  <li><code>calculo_error_posicion()</code>: Retorna el error en metros.</li>
  <li><code>calcular_error_orientacion()</code>: Retorna el error en grados.</li>
</ul>

<h2>📜 Licencia</h2>

<p>Este proyecto está licenciado bajo los términos de la MIT License.</p>

<h2>🤝 Contribuciones</h2>

<p>Si otro desarrollador ha participado en este proyecto, se recomienda:</p>
<ul>
  <li>Incluirlo en la sección de <code>Contributors</code> del README.</li>
  <li>Agregarlo como colaborador en GitHub desde la pestaña "Settings" → "Collaborators".</li>
</ul>
