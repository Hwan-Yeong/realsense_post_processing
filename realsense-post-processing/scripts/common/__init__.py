"""rs_floor_ransac_test.py / rs_ransac_tuner.py 공용 로직.

CLAUDE.md의 "공통 로직이 스크립트마다 중복됨 -> scripts/common/으로 분리 권장"에 따라 분리했다.

중요: RANSAC은 여기에 구현되어 있지 않다. Python은 알고리즘을 갖지 않고
C++ 코어(floor_ransac_py)를 호출하기만 한다 — 튜너로 찾은 파라미터가
ROS 노드/임베디드에서 그대로 재현되게 하려면 구현이 하나여야 하기 때문이다.
core.py 문서 참고.
"""
