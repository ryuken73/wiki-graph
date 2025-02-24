import React from 'react';
import ButtonSmall from './Common/ButtonSmall';
import styled from 'styled-components';
const Container = styled.div`
  position: absolute;
  top: 10px;
  right: 10px;
`

function QuickButtons(props) {
  const {removeAllNodes} = props;
  return (
    <Container>
      <ButtonSmall onClick={removeAllNodes} fontSize='16px'>전체삭제</ButtonSmall>
    </Container>
  )
}

export default React.memo(QuickButtons)