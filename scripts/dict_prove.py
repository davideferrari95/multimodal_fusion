
oggetti={
            
            "il sale grosso":[
            [2.5016584396362305, -0.8632076543620606, 1.2751339117633265, -1.963386674920553, 4.741418838500977, 0.9712827205657959],
            ],

            "il sale iodato":[
            [3.387843608856201, -1.0682671827128907, 1.6490314642535608, -2.1364666424193324, 4.772143363952637, 1.8173348903656006],
            ],

            "il bicchiere bianco":[
            [2.3846733570098877, -1.0724294942668458, 1.6509855429278772, -2.178251882592672, 4.730436325073242, 0.85353684425354],
            ],

            "la passata":[
            [2.3381829261779785, -0.8568876546672364, 1.3088882605182093, -2.0508209667601527, 4.731287956237793, 0.8080871105194092],
            ], 

            "il coltello":[
            [2.2267518043518066, -1.1784547132304688, 2.059087578450338, -2.477241178552145, 4.735124588012695, 0.6946213245391846],
            ], 

            "la forchetta di legno":[
            [2.293628692626953, -1.0718673032573243, 1.85292894044985, -2.3308073482909144, 4.7392730712890625, -0.8873665968524378],
            ], 

            "le pennette":[
            [2.1489624977111816, -1.074343041782715, 1.6118515173541468, -2.0838972530760707, 4.7358078956604, -1.0315120855914515],
            ], 

            "la forchetta di plastica":[
            [3.3018717765808105, -1.2413643163493653, 2.0898597876178187, -2.3902908764281214, 4.729753017425537, -1.270754639302389],
            ], 

            "i pelati":[
            [3.3865585327148438, -1.4877928060344239, 2.2749279181109827, -2.3304363689818324, 4.7321391105651855, -1.1869919935809534],
            ], 

            "il bicchiere trasparente":[
            [3.4324843883514404, -1.2404654783061524, 1.9565780798541468, -2.2602154217162074, 4.73328971862793, -1.139892880116598],
            ],

            "gli spaghetti":[
            [3.595914840698242, -1.1588075918010254, 1.9754837195025843, -2.4072934589781703, 4.74080753326416, 0.44310879707336426],
            ]
                    }
    


area_destra = {
        "la passata": 11,
        "il sale grosso": 21, 
        "il bicchiere bianco": 61,
        "le pennette": 31,
        "il coltello": 51,
        "forchetta di legno": 41,
    }
area_sinistra = {
        "il sale iodato": 22,
        "il bicchiere trasparente": 62,
        "gli spaghetti": 32,
        "la forchetta di plastica": 42,
        "i pelati": 12, 
    }



# value_to_find = 21


# for key, value in area_destra.items():
#         if value == value_to_find:
#             print("Primo valore trovato nella dictionary 'self.area_destra':", key)
#             coordinate = oggetti[key]
#             print(coordinate)
#             break

# for key, value in area_sinistra.items():
#     if value == value_to_find:
#         print("Primo valore trovato nella dictionary 'self.area_sinistra':", key)
#         coordinate = oggetti[key]
#         print(coordinate)

#         break



print('Nell\'area di destra ci sono: {}'.format(', '.join(area_destra)))