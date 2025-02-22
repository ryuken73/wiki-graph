import React from 'react';
import styled from 'styled-components';
// import ButtonSmall from './Common/ButtonSmall';
const Container = styled.div`
  padding-left: 10px;
`
const ButtonSmall = styled.div`
  text-align: left;
  font-weight: 200;
  font-size: 12px;
  color: yellow;
  cursor: pointer;
  &:hover {
    color: white;
    font-weight: 300;
  }
`

export default function ActionButtons() {
  return (
    <Container>
      <ButtonSmall fontSize='12px' color='black' background='white'>즐겨찾기</ButtonSmall>
      <ButtonSmall fontSize='12px' color='black' background='white'>삭제</ButtonSmall>
      <ButtonSmall fontSize='12px' color='black' background='white'>확장(IN)</ButtonSmall>
      <ButtonSmall fontSize='12px' color='black' background='white'>확장(OUT)</ButtonSmall>
      <ButtonSmall fontSize='12px' color='black' background='white'>축소(IN)</ButtonSmall>
      <ButtonSmall fontSize='12px' color='black' background='white'>축소(OUT)</ButtonSmall>
      <ButtonSmall fontSize='12px' color='black' background='white'>참조</ButtonSmall>

    </Container>
  )
}
