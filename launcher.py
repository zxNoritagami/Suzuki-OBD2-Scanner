#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LAUNCHER - Bootstrap de Suzuki OBD2 Scanner Pro
------------------------------------------------
Gestiona automaticamente el entorno virtual y las dependencias
antes de lanzar la aplicacion principal.

USO:
    python launcher.py
    python launcher.py --simulate
"""

import sys
import os
import subprocess
import venv
from pathlib import Path
from typing import List, Optional

# ==============================================================================
# CONSTANTES
# ==============================================================================

APP_DIR = Path(__file__).resolve().parent
VENV_DIR = APP_DIR / "venv"
REQUIREMENTS_FILE = APP_DIR / "requirements.txt"
MAIN_SCRIPT = APP_DIR / "suzuki_obd2_scanner.py"

# En Windows, el ejecutable de Python del venv esta en Scripts/
if sys.platform == "win32":
    VENV_PYTHON = VENV_DIR / "Scripts" / "python.exe"
    VENV_PIP = VENV_DIR / "Scripts" / "pip.exe"
else:
    VENV_PYTHON = VENV_DIR / "bin" / "python"
    VENV_PIP = VENV_DIR / "bin" / "pip"

# ==============================================================================
# UTILIDADES
# ==============================================================================

def _print_step(message: str):
    """Imprime un paso del bootstrap con formato limpio."""
    print(f"  [{chr(0x2504)}] {message}")


def _print_ok(message: str):
    print(f"  [{chr(0x2713)}] {message}")


def _print_error(message: str):
    print(f"  [{chr(0x2717)}] {message}")


def _print_warn(message: str):
    print(f"  [!] {message}")


# ==============================================================================
# VERIFICACION DEL ENTORNO VIRTUAL
# ==============================================================================

def _check_venv_exists() -> bool:
    """Verifica si el entorno virtual ya existe."""
    return VENV_DIR.exists() and (VENV_PYTHON.exists() or (VENV_DIR / "pyvenv.cfg").exists())


def _create_venv() -> bool:
    """Crea el entorno virtual."""
    try:
        print()
        _print_step("Creando entorno virtual...")
        builder = venv.EnvBuilder(
            with_pip=True,
            clear=False,
            symlinks=False if sys.platform == "win32" else True,
        )
        builder.create(str(VENV_DIR))
        _print_ok("Entorno virtual creado en: venv/")
        return True
    except Exception as e:
        _print_error(f"No se pudo crear el entorno virtual: {e}")
        return False


# ==============================================================================
# VERIFICACION E INSTALACION DE DEPENDENCIAS
# ==============================================================================

def _get_installed_packages() -> List[str]:
    """Obtiene la lista de paquetes instalados en el venv."""
    try:
        result = subprocess.run(
            [str(VENV_PYTHON), "-m", "pip", "list", "--format=freeze"],
            capture_output=True, text=True, timeout=30
        )
        if result.returncode != 0:
            return []
        return [line.strip().lower() for line in result.stdout.strip().splitlines() if line.strip()]
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []


def _read_requirements() -> List[str]:
    """Lee el archivo requirements.txt."""
    if not REQUIREMENTS_FILE.exists():
        _print_warn(f"No se encontro {REQUIREMENTS_FILE.name}. Creando archivo por defecto...")
        _create_default_requirements()
    
    with open(REQUIREMENTS_FILE, "r", encoding="utf-8") as f:
        return [
            line.strip()
            for line in f
            if line.strip() and not line.startswith("#")
        ]


def _create_default_requirements():
    """Crea un requirements.txt por defecto si no existe."""
    with open(REQUIREMENTS_FILE, "w", encoding="utf-8") as f:
        f.write("# Suzuki OBD2 Scanner Pro - Dependencias\n")
        f.write("# Instalacion automatica via: python launcher.py\n\n")
        f.write("pyserial>=3.5\n")
        f.write("customtkinter>=5.2.2\n")
        f.write("pillow>=10.0.0\n")
    _print_ok(f"{REQUIREMENTS_FILE.name} creado con valores por defecto.")


def _get_missing_packages(required: List[str], installed: List[str]) -> List[str]:
    """Compara paquetes requeridos vs instalados y devuelve los faltantes."""
    missing = []
    installed_lower = [pkg.split("==")[0].lower() for pkg in installed]
    
    for req in required:
        # Normalizar: quitar versiones (>=, ==, <=, ~=, !=)
        pkg_name = req.split(">=")[0].split("==")[0].split("<=")[0].split("~=")[0].strip().lower()
        if pkg_name and pkg_name not in installed_lower and not pkg_name.startswith("#"):
            missing.append(req)
    
    return missing


def _install_packages(packages: List[str]) -> bool:
    """Instala los paquetes faltantes en el venv."""
    if not packages:
        return True

    print()
    _print_step(f"Instalando {len(packages)} dependencia(s)...")
    
    for pkg in packages:
        print(f"    -> {pkg}")
    
    try:
        # Primero actualizar pip
        subprocess.run(
            [str(VENV_PYTHON), "-m", "pip", "install", "--upgrade", "pip"],
            capture_output=True, timeout=60
        )
        
        # Instalar paquetes
        result = subprocess.run(
            [str(VENV_PYTHON), "-m", "pip", "install"] + packages,
            capture_output=True, text=True, timeout=120
        )
        
        if result.returncode == 0:
            _print_ok("Dependencias instaladas correctamente.")
            return True
        else:
            _print_error("Error instalando dependencias:")
            for line in result.stderr.strip().splitlines():
                print(f"      {line}")
            return False
            
    except subprocess.TimeoutExpired:
        _print_error("Tiempo de instalacion agotado. Verifica tu conexion a internet.")
        return False
    except Exception as e:
        _print_error(f"Error inesperado: {e}")
        return False


# ==============================================================================
# LANZAMIENTO DE LA APLICACION PRINCIPAL
# ==============================================================================

def _launch_main(simulate: bool = False):
    """Lanza la aplicacion principal usando el interprete del venv."""
    if not MAIN_SCRIPT.exists():
        _print_error(f"No se encuentra el archivo principal: {MAIN_SCRIPT.name}")
        _print_step(f"Asegurate de que {MAIN_SCRIPT.name} este en el mismo directorio que {__file__}")
        sys.exit(1)
    
    args = [str(VENV_PYTHON), str(MAIN_SCRIPT)]
    if simulate:
        args.append("--simulate")
    
    print()
    print(f"  {chr(0x2501)}" * 40)
    print(f"  Iniciando Suzuki OBD2 Scanner Pro...")
    if simulate:
        print(f"  Modo: SIMULACION (sin hardware)")
    print(f"  {chr(0x2501)}" * 40)
    print()
    
    try:
        if sys.platform == "win32":
            # En Windows, usar creationflags para ocultar la ventana de cmd
            # si se desea; por defecto mantenemos visible para debug
            process = subprocess.Popen(
                args,
                cwd=str(APP_DIR),
                shell=False,
            )
        else:
            process = subprocess.Popen(
                args,
                cwd=str(APP_DIR),
            )
        
        # Esperar a que termine la app principal
        process.wait()
        
    except FileNotFoundError:
        _print_error(f"No se encuentra el interprete de Python en el entorno virtual.")
        _print_step(f"Verifica que exista: {VENV_PYTHON}")
        sys.exit(1)
    except Exception as e:
        _print_error(f"Error al lanzar la aplicacion: {e}")
        sys.exit(1)


# ==============================================================================
# LIMPIEZA (OPCIONAL - LOGS TEMPORALES, ETC.)
# ==============================================================================

def _cleanup_temp():
    """Limpia archivos temporales generados durante el bootstrap."""
    # Por ahora placeholder - en futuras versiones puede limpiar
    # logs de instalacion, archivos .tmp, etc.
    pass


# ==============================================================================
# PUNTO DE ENTRADA DEL BOOTSTRAPPER
# ==============================================================================

def main():
    """Punto de entrada del launcher."""
    # Parsear argumentos basicos (--simulate)
    simulate = "--simulate" in sys.argv
    
    print()
    print(f"  {chr(0x2501)}" * 40)
    print(f"  Suzuki OBD2 Scanner Pro - Launcher")
    print(f"  v1.0.0 | Windows / Python")
    print(f"  {chr(0x2501)}" * 40)
    
    # ------------------------------------------------------------------
    # PASO 1: Verificar / Crear entorno virtual
    # ------------------------------------------------------------------
    print()
    print(f"  [{chr(0x2501)}] Entorno virtual")
    
    if _check_venv_exists():
        _print_ok("Entorno virtual encontrado en: venv/")
    else:
        _print_step("No se encontro entorno virtual.")
        if not _create_venv():
            _print_error("No se puede continuar sin el entorno virtual.")
            print()
            print("  Posibles soluciones:")
            print("    1. Verifica que tengas permisos de escritura en este directorio")
            print("    2. Crea el entorno manualmente: python -m venv venv")
            print("    3. Ejecuta como administrador si es necesario")
            print()
            sys.exit(1)
    
    # ------------------------------------------------------------------
    # PASO 2: Verificar / Instalar dependencias
    # ------------------------------------------------------------------
    print()
    print(f"  [{chr(0x2501)}] Dependencias")
    
    required = _read_requirements()
    installed = _get_installed_packages()
    missing = _get_missing_packages(required, installed)
    
    if missing:
        _print_step(f"Faltan {len(missing)} dependencia(s) por instalar.")
        if not _install_packages(missing):
            _print_error("No se pudieron instalar todas las dependencias.")
            print()
            print("  Posibles soluciones:")
            print("    1. Verifica tu conexion a internet")
            print("    2. Instala manualmente: venv\\Scripts\\pip install -r requirements.txt")
            print("    3. Verifica que no haya restricciones de firewall/proxy")
            print()
            sys.exit(1)
    else:
        _print_ok("Todas las dependencias estan instaladas.")
    
    # ------------------------------------------------------------------
    # PASO 3: Lanzar aplicacion principal
    # ------------------------------------------------------------------
    print()
    print(f"  [{chr(0x2501)}] Lanzamiento")
    _launch_main(simulate=simulate)
    
    # ------------------------------------------------------------------
    # PASO 4: Limpieza post-cierre
    # ------------------------------------------------------------------
    _cleanup_temp()
    
    print()
    _print_ok("Aplicacion cerrada correctamente.")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        print("  [!] Operacion cancelada por el usuario.")
        sys.exit(0)
    except Exception as e:
        print()
        _print_error(f"Error inesperado en el launcher: {e}")
        print()
        print("  Detalles del error:")
        import traceback
        traceback.print_exc()
        print()
        print("  Si el problema persiste, ejecuta manualmente:")
        print(f"    python {MAIN_SCRIPT.name} --simulate")
        print()
        sys.exit(1)
