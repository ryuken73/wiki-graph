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

export default function ActionButtons(props) {
  const {
    nodeHovered,
    expandNode,
    removeNode
  } = props;
  const nodeId = nodeHovered.id;
  const removeNodeHovered = React.useCallback(() => {removeNode(nodeId)}, [nodeId]);
  const expandNodeIn = React.useCallback(() => {expandNode(nodeHovered, false)}, [nodeHovered]);
  const expnadNodeOut = React.useCallback(() => {expandNode(nodeHovered, true)}, [nodeHovered]);
  const shrinkNodeIn = React.useCallback(() => {}, []);
  const shrinkNodeOut = React.useCallback(() => {}, []);
  const showAllIn = React.useCallback(() => {}, []);
  const addFavorate = React.useCallback(() => {}, []);
  return (
    <Container>
      <ButtonSmall onClick={removeNodeHovered} fontSize='12px' color='black' background='white'>삭제</ButtonSmall>
      <ButtonSmall onClick={expandNodeIn}fontSize='12px' color='black' background='white'>확장(IN)</ButtonSmall>
      <ButtonSmall onClick={expnadNodeOut} fontSize='12px' color='black' background='white'>확장(OUT)</ButtonSmall>
      <ButtonSmall onClick={shrinkNodeIn} fontSize='12px' color='black' background='white'>축소(IN)</ButtonSmall>
      <ButtonSmall onClick={shrinkNodeOut} fontSize='12px' color='black' background='white'>축소(OUT)</ButtonSmall>
      <ButtonSmall onClick={showAllIn} fontSize='12px' color='black' background='white'>참조</ButtonSmall>
      <ButtonSmall onClick={addFavorate} fontSize='12px' color='black' background='white'>즐겨찾기</ButtonSmall>
    </Container>
  )
}
