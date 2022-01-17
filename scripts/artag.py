# ALL MEASUREMENTS ARE IN CM

AREA_DIMENSIONS = (
    580,  # x
    290,  # y
    300,  # z
)
INITIAL_POSITION = (290, 145, 0)

AR_TAGS = {  # id: position of the ar tag
    1: (310, 0, 140),
}


def getPosition(artag_id_1, artag_d_1,
                artag_id_2, artag_d_2,
                artag_id_3, artag_d_3, ):
    distance_sum = artag_d_1 + artag_d_2 + artag_d_3
    res = [0, 0, 0]
    for coord_index in range(len(res)):
        res[coord_index] = AR_TAGS[artag_id_1][coord_index] * artag_d_1 + AR_TAGS[artag_id_2][coord_index] * artag_d_2 + AR_TAGS[artag_id_3][coord_index] * artag_d_3
        res[coord_index] = res[coord_index] / distance_sum
    return tuple(res)


if __name__ == '__main__':
    pass
