
import tkinter as tk
from tkinter import PhotoImage
import subprocess
import os

# Caminhos absolutos dos scripts


def executar_cod1():
    subprocess.Popen(["contr_scara.exe"], shell=True)
def executar_cod2():
    subprocess.Popen(["cubos_scara.exe"], shell=True)

def fechar():
    janela.destroy()

# Criar janela principal maior
janela = tk.Tk()
janela.title("Controle do Robô SCARA")
janela.geometry("800x600")
janela.configure(bg="#f0f0f0")

# Carregar imagem (formato PNG recomendado)
try:
    imagem = PhotoImage(file="manipulador.png")  # coloque a imagem na mesma pasta
    label_img = tk.Label(janela, image=imagem, bg="#f0f0f0")
    label_img.pack(pady=20)
except Exception as e:
    label_img = tk.Label(janela, text="[Imagem não encontrada]", bg="#f0f0f0", font=("Arial", 16))
    label_img.pack(pady=20)

# Botões para iniciar SCARA5 e SCARA6
botao1 = tk.Button(janela, text="Iniciar Movimentação Manual", font=("Arial", 16), width=25, height=2, command=executar_cod1)
botao1.pack(pady=10)

botao2 = tk.Button(janela, text="Iniciar Automação", font=("Arial", 16), width=25, height=2, command=executar_cod2)
botao2.pack(pady=10)

# Botão menor para sair
botao_sair = tk.Button(janela, text="Fechar", font=("Arial", 12), width=10, command=fechar)
botao_sair.pack(side="bottom", pady=20)

# Loop principal
janela.mainloop()

'''
import tkinter as tk
from tkinter import PhotoImage
import subprocess
import os
import sys

# Caminhos absolutos dos scripts .py
CAMINHO_COD1 = os.path.abspath("contr_scara.py")
CAMINHO_COD2 = os.path.abspath("cubos_scara.py")

def executar_cod1():
    subprocess.Popen([sys.executable, CAMINHO_COD1], shell=True)

def executar_cod2():
    subprocess.Popen([sys.executable, CAMINHO_COD2], shell=True)

def fechar():
    janela.destroy()

# Criar janela principal maior
janela = tk.Tk()
janela.title("Controle do Robô SCARA")
janela.geometry("800x600")
janela.configure(bg="#f0f0f0")

# Carregar imagem (formato PNG recomendado)
try:
    imagem = PhotoImage(file="manipulador.png")  # coloque a imagem na mesma pasta
    label_img = tk.Label(janela, image=imagem, bg="#f0f0f0")
    label_img.pack(pady=20)
except Exception as e:
    label_img = tk.Label(janela, text="[Imagem não encontrada]", bg="#f0f0f0", font=("Arial", 16))
    label_img.pack(pady=20)

# Botões para iniciar os códigos
botao1 = tk.Button(janela, text="Iniciar Controle", font=("Arial", 16), width=20, height=2, command=executar_cod1)
botao1.pack(pady=10)

botao2 = tk.Button(janela, text="Iniciar Automação", font=("Arial", 16), width=20, height=2, command=executar_cod2)
botao2.pack(pady=10)

# Botão menor para sair
botao_sair = tk.Button(janela, text="Fechar", font=("Arial", 12), width=10, command=fechar)
botao_sair.pack(side="bottom", pady=20)

# Loop principal
janela.mainloop()
'''