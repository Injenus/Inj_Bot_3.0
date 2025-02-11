import numpy as np


main_array = np.arange(-120, 121, 10)
main_array = np.insert(main_array, 0, -128)
main_array = np.append(main_array, 127)


len_m90 = 120
array_m90 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m90, -170], [-len_m90, 0], [0,0], [len_m90, 0], [len_m90, -170], [np.inf, -170], [np.inf, -np.inf]])
# ----------------------------------------------------------------------------
len_m100 = len_m90/np.sin(np.radians(100))
array_m100 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m100, -170], [-len_m100, 0], [0,0], [len_m100, 0], [len_m100, -170], [np.inf, -170], [np.inf, -np.inf]])
len_m110 = len_m90/np.sin(np.radians(110))
array_m110 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m110, -170], [-len_m110, 0], [0,0], [len_m110, 0], [len_m110, -170], [np.inf, -170], [np.inf, -np.inf]])
len_m120 = len_m90/np.sin(np.radians(120))
array_m120 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m120, -170], [-len_m120, 0], [0,0], [len_m120, 0], [len_m120, -170], [np.inf, -170], [np.inf, -np.inf]])
len_m128 = len_m90/np.sin(np.radians(128))
array_m128 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m128, -170], [-len_m128, 0], [0,0], [len_m128, 0], [len_m128, -170], [np.inf, -170], [np.inf, -np.inf]])
# ----------------------------------------------------------------------------
len_m80 = len_m90/np.sin(np.radians(80))
array_m80 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m80, -170], [-len_m80, 0], [0,0], [len_m80, 0], [len_m80, -170], [np.inf, -170], [np.inf, -np.inf]])
len_m70 = len_m90/np.sin(np.radians(70))
array_m70 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m70, -170], [-len_m70, 0], [0,0], [len_m70, 0], [len_m70, -170], [np.inf, -170], [np.inf, -np.inf]])
len_m60 = len_m90/np.sin(np.radians(60))
array_m60 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m60, -170], [-len_m60, 0], [0,0], [len_m60, 0], [len_m60, -170], [np.inf, -170], [np.inf, -np.inf]])
len_m50 = len_m90/np.sin(np.radians(50))
array_m50 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_m50, -170], [-len_m50, 0], [0,0], [len_m50, 0], [len_m50, -170], [np.inf, -170], [np.inf, -np.inf]])
# ----------------------------------------------------------------------------
len_m40_back, len_m40_forw = 180, 200
array_m40 = np.array([[[-np.inf, -np.inf], [-np.inf, -170], [-len_m40_back, -170], [-len_m40_back, 0], [0, 0], [len_m40_forw, 0], [len_m40_forw, -170], [np.inf, -170], [np.inf, np.inf]]])
array_m30 = np.copy(array_m40)
array_m20 = np.copy(array_m30)
# ----------------------------------------------------------------------------
len_10m_back, len_10m_forw = 180, 210
array_m10 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_10m_back, -170], [-len_10m_back, 0], [0, 0], [len_10m_forw, 0], [len_10m_forw, -170] ,[np.inf, -170], [np.inf, np.inf]])
array_0 = np.copy(array_m10)
array_p10 = np.copy(array_0)
# ----------------------------------------------------------------------------
array_p40 = np.copy(array_m40)
array_p30 = np.copy(array_p40)
array_p20 = np.copy(array_p30)
# ----------------------------------------------------------------------------
array_p50 = np.copy(array_m50)
array_p60 = np.copy(array_m60)
array_p70 = np.copy(array_m70)
array_p80 = np.copy(array_m80)
array_p90 = np.copy(array_m90)
array_p100 = np.copy(array_m100)
array_p110 = np.copy(array_m110)
array_p120 = np.copy(array_m120)
len_p127 = len_m90/np.sin(np.radians(127))
array_p127 = np.array([[-np.inf, -np.inf], [-np.inf, -170], [-len_p127, -170], [-len_p127, 0], [0,0], [len_p127, 0], [len_p127, -170], [np.inf, -170], [np.inf, -np.inf]])


arrays = [array_m128,
          array_m120,
          array_m110,
          array_m100,
          array_m90,
          array_m80,
          array_m70,
          array_m60,
          array_m50,
          array_m40,
          array_m30,
          array_m20,
          array_m10,
          array_0,
          array_p10,
          array_p20,
          array_p30,
          array_p40,
          array_p50,
          array_p60,
          array_p70,
          array_p80,
          array_p90,
          array_p100,
          array_p110,
          array_p120,
          array_p127,
          ]

if len(arrays) != main_array.shape[0]:
    assert len(arrays) != main_array.shape[0], "Lengths are not equal!"

# Сохраняем данные в сжатом формате NPZ
np.savez_compressed(
    'valid_area_data.npz',
    main=main_array,
    **{f'array_{i}': arr for i, arr in enumerate(arrays)}
)

if __name__ == "__main__":
    loaded_data = np.load('valid_area_data.npz')
    # Извлекаем основной массив
    main_loaded = loaded_data['main']
    # Восстанавливаем список массивов, сохраняя порядок
    arrays_loaded = [loaded_data[f'array_{i}'] for i in range(len(main_loaded))]

    # Сопоставление i-го элемента с i-м массивом
    for i, (value, arr) in enumerate(zip(main_loaded, arrays_loaded)):
        print(f"Элемент {i}: значение = {value}, массив = {arr}")


